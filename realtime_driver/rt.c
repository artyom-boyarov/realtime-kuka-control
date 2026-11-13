#define _GNU_SOURCE
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

// For UDP networking.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define PERIOD_NS 100000L
#define NUM_JOINTS 7
#define SENSOR_PORT 7201
#define COMMAND_PORT 7200
// If no packet arrives within this long, the feedback is considered stale -
// the control loop should fail safe rather than act on an old snapshot.
// A gap this long means the other side (the simulation) has gone away, not
// just an isolated dropped packet - exit rather than keep waiting.
#define SENSOR_TIMEOUT_MS 250

#ifdef RECORD_LAT
#define N_LOOPS 10000 // 10,000 loops to get data
#define LATENCY_LOG_PATH "control_loop_latency.bin"
#endif

// PD constants mirror pybullet_sim/pybullet_kuka_iiwa/pd_client.py's
// --kp/--kd defaults, and TARGET mirrors its TARGET pose.
#define KP 500.0f
#define KD 400.0f
#define DEG2RAD(d) ((float) ((d) * 3.14159265358979323846 / 180.0))

static const float TARGET[NUM_JOINTS] = {
    DEG2RAD(-103.67),
    DEG2RAD(-59.22),
    DEG2RAD(13.27),
    DEG2RAD(81.81),
    DEG2RAD(-43.18),
    DEG2RAD(53.98),
    DEG2RAD(69.82),
};

// Per-joint torque limit, from each joint's <limit effort="..."> in
// lbr_iiwa7_r800.urdf (joint1..joint7).
static const float MAX_TORQUE[NUM_JOINTS] = {
    176.0f,
    176.0f,
    110.0f,
    110.0f,
    110.0f,
    40.0f,
    40.0f,
};

static void ts_add(struct timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000L;
    }
}

// Shared buffer between the sensor thread (writer) and the control thread
// (reader). Protected by a mutex since each joint-angle packet is written
// and read as a whole 7-float snapshot.
typedef struct {
    pthread_mutex_t mutex;
    float angles[NUM_JOINTS];
} joint_state_t;

static joint_state_t g_joint_state = {
    .mutex = PTHREAD_MUTEX_INITIALIZER,
};

// Structure
// Real-time loop for computing PD feedback and sending it over UDP.
// Second thread for receiving current joint angles - writes to a shared buffer.
static void* sensor_loop(void* arg) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return NULL;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SENSOR_PORT),
    };
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (bind(sock, (struct sockaddr *) &addr, sizeof addr) < 0) {
        perror("bind");
        close(sock);
        return NULL;
    }

    // Bound the blocking recvfrom() below so a gap in incoming packets (the
    // other side died, network hiccup, ...) is itself observable instead of
    // just hanging - that gap is what marks the shared state as stale.
    struct timeval recv_timeout = {
        .tv_sec = SENSOR_TIMEOUT_MS / 1000,
        .tv_usec = (SENSOR_TIMEOUT_MS % 1000) * 1000,
    };
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof recv_timeout) < 0) {
        perror("setsockopt timeout");
        close(sock);
        return NULL;
    }

    float packet[NUM_JOINTS];
    for (;;) {
        ssize_t received = recvfrom(sock, packet, sizeof packet, 0, NULL, NULL);
        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No packet within the timeout - assume the simulation shut
                // down and exit the loop.
                break;
            }
            perror("recvfrom");
            continue;
        }
        if ((size_t) received != sizeof packet) {
            // Malformed packet - wrong number of joints.
            continue;
        }

        pthread_mutex_lock(&g_joint_state.mutex);
        memcpy(g_joint_state.angles, packet, sizeof packet);
        pthread_mutex_unlock(&g_joint_state.mutex);
    }

    close(sock);
    return NULL;
}


static void *control_loop(void* arg) {
    (void) arg;

#ifdef RECORD_LAT
    long *latency_measurements = malloc((size_t) N_LOOPS * sizeof(long));
    if (latency_measurements == NULL) {
        perror("malloc");
        return NULL;
    }
#endif
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return NULL;
    }
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(COMMAND_PORT),
    };
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    // Previous snapshot for the finite-difference velocity estimate below.
    float previous[NUM_JOINTS] = {0};
    int have_previous = 0;
    const double period_s = PERIOD_NS / 1e9;

    struct timespec next, now;
    long max_lat = 0;

    // Monotonic clock cannot be set - is monotonically increasing from some
    // prior starting point.
    clock_gettime(CLOCK_MONOTONIC, &next);

    // Run for 1s
#ifdef RECORD_LAT
    for (long i = 0; i < N_LOOPS; i++) {
#else
    for ( ;; ) {
#endif
        ts_add(&next, PERIOD_NS);

        // At each loop, next becomes the end of the execution loop. The clock
        // is at start + do_work() time. So we sleep for the remaining time in
        // the period, until next (absolute time) which is always incremented
        // by the period.
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);


        // Read sensors (shared memory/UDP) -> compute control.
        float angles[NUM_JOINTS];
        pthread_mutex_lock(&g_joint_state.mutex);
        memcpy(angles, g_joint_state.angles, sizeof angles);
        pthread_mutex_unlock(&g_joint_state.mutex);

        float torque[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++) {
            // Speed from the period-fixed difference between control ticks,
            // not wall-clock time, since the loop itself runs at PERIOD_NS.
            float velocity = have_previous
                ? (float) ((angles[i] - previous[i]) / period_s)
                : 0.0f;
            float command = -KP * (angles[i] - TARGET[i]) - KD * velocity;
            if (command > MAX_TORQUE[i]) command = MAX_TORQUE[i];
            if (command < -MAX_TORQUE[i]) command = -MAX_TORQUE[i];
            torque[i] = command;
        }
        memcpy(previous, angles, sizeof previous);
        have_previous = 1;

        sendto(sock, torque, sizeof torque, 0, (struct sockaddr *) &addr, sizeof addr);


        clock_gettime(CLOCK_MONOTONIC, &now);
        long lat = (now.tv_sec - next.tv_sec) * 1000000000L + (now.tv_nsec - next.tv_nsec);
        if (lat > max_lat) max_lat = lat;

#ifdef RECORD_LAT
        latency_measurements[i] = lat;
#endif
    }

#ifdef RECORD_LAT
    // Dump the recorded per-iteration latencies (ns, native long/endianness)
    // as raw binary for offline analysis - see plot_latency.py.
    FILE *log = fopen(LATENCY_LOG_PATH, "wb");
    if (log == NULL) {
        perror("fopen");
    } else {
        size_t written = fwrite(latency_measurements, sizeof(long), N_LOOPS, log);
        if (written != (size_t) N_LOOPS) {
            perror("fwrite");
        }
        fclose(log);
    }
    free(latency_measurements);
#endif

    close(sock);
    return NULL;
}

int main(void) {
    // Stops page faults - ensures memory pages are allocated and available
    // Locks every currently allocated pages and any future allocated pages
    
    // Try not locking memory...
    mlockall(MCL_CURRENT | MCL_FUTURE);

    {
        volatile char stack[256 * 1024];
        memset((char *) stack, 0, sizeof stack);
    } // Prefault memory

    pthread_attr_t sensor_thread_attr, control_thread_attr;

    struct sched_param sensor_sp = { .sched_priority = 70 };
    struct sched_param control_sp = { .sched_priority = 70 };
    cpu_set_t sensor_cpus, control_cpus;
    CPU_ZERO(&sensor_cpus);
    CPU_SET(3, &sensor_cpus);
    CPU_ZERO(&control_cpus);
    CPU_SET(3, &control_cpus);

    pthread_attr_init(&sensor_thread_attr);
    // Inherit - thread created inherits scheduling attributes from calling
    // thread.
    // Explicit - use scheduling attributed in pthread_attr_t* object.
    pthread_attr_setinheritsched(&sensor_thread_attr, PTHREAD_EXPLICIT_SCHED);
    // Sets scheduling policy.
    pthread_attr_setschedpolicy(&sensor_thread_attr, SCHED_FIFO);
    // Sets parameters (only the scheduler priority)
    pthread_attr_setschedparam(&sensor_thread_attr, &sensor_sp);
    // Sets thread's CPU affinity mask - which CPUs the thread can run on.
    pthread_attr_setaffinity_np(&sensor_thread_attr, sizeof sensor_cpus, &sensor_cpus);

    pthread_attr_init(&control_thread_attr);
    pthread_attr_setinheritsched(&control_thread_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&control_thread_attr, SCHED_FIFO);
    pthread_attr_setschedparam(&control_thread_attr, &control_sp);
    pthread_attr_setaffinity_np(&control_thread_attr, sizeof control_cpus, &control_cpus);

    // Configure priority inheritance
    pthread_mutexattr_t joint_mutex_attr;
    pthread_mutexattr_init(&joint_mutex_attr);
    pthread_mutexattr_setprotocol(&joint_mutex_attr, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&g_joint_state.mutex, &joint_mutex_attr);

    pthread_t sensor_thread, control_thread;
    int err;
    if ((err = pthread_create(&sensor_thread, &sensor_thread_attr, sensor_loop, NULL)) != 0) {
        fprintf(stderr, "pthread_create: %s\n", strerror(err));
        return 1;
    }
    if ((err = pthread_create(&control_thread, &control_thread_attr, control_loop, NULL)) != 0) {
        fprintf(stderr, "pthread_create: %s\n", strerror(err));
        return 1;
    }

    // sensor_loop returns once feedback goes quiet for SENSOR_TIMEOUT_MS,
    // i.e. the simulation shut down - stop the control loop too.
    pthread_join(sensor_thread, NULL);
    pthread_cancel(control_thread);
    pthread_join(control_thread, NULL);

    return 0;
}