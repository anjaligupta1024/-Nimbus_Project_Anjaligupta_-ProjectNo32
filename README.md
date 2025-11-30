# -Nimbus_Project_Anjaligupta_-ProjectNo32
Adaptive Traffic Intersection Controller 
/* adaptive_traffic.c
   Adaptive Traffic Intersection Controller (Sensor Driven)
   Features:
   - Dynamic approaches (add/remove)
   - 2D lane mapping
   - Adaptive green allocation proportional to queue lengths (with min/max)
   - Fixed-time baseline for comparison
   - Event logging to file
   - Performance metrics: avg wait time, throughput, avg queue
   - Menu-driven console application
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

/* ---------- CONFIGURABLE DEFAULTS ---------- */
#define MAX_NAME 32
#define DEFAULT_MIN_GREEN 10  /* seconds */
#define DEFAULT_MAX_GREEN 60  /* seconds */
#define DEFAULT_CYCLE 120     /* total cycle time in seconds */
#define DEFAULT_ALL_RED 5     /* seconds reserved for clearance per cycle */
#define MAX_LANES 8

/* ---------- STRUCTS ---------- */

typedef struct {
    int id;                          /* approach id */
    char name[MAX_NAME];             /* name or direction */
    int lanes;                       /* number of lanes */
    double arrival_rate;             /* vehicles per second (input/sensor) */
    double service_rate;             /* vehicles per second when green (saturation) */
    int queue;                       /* current queue length (vehicles) */
    double cumulative_wait;          /* sum of wait times for vehicles */
    long total_arrived;              /* total vehicles arrived during sim */
    long total_served;               /* total vehicles served */
    double cum_queue_length;         /* for average queue computation (sum over time steps) */
} Approach;

typedef struct {
    int time_sec;
    int approach_id;     /* which approach had green at this second (-1 = none) */
    int vehicles_passed; /* vehicles passed in that second */
} EventLogEntry;

typedef struct {
    double avg_wait_time;
    long throughput;
    double avg_queue_length;
} Metrics;

/* ---------- GLOBAL SIMULATION STATE ---------- */

Approach **approaches = NULL;  /* pointer array to Approach pointers */
int num_approaches = 0;

/* Event log dynamic array */
EventLogEntry *event_log = NULL;
size_t event_log_len = 0;
size_t event_log_capacity = 0;

/* Random helper */
static unsigned int rng_seed;

/* ---------- UTILITIES ---------- */

void seed_rng() {
    rng_seed = (unsigned int) time(NULL);
    srand(rng_seed);
}

/* Poisson-ish random arrivals: approximate Poisson by sampling uniform draws for simplicity.
   For small dt=1s, arrival_rate can be used to sample expected arrivals using exponential or rounding.
   We'll use a simple Poisson generator via Knuth algorithm. */
int poisson_random(double lambda) {
    if (lambda <= 0.0) return 0;
    double L = exp(-lambda);
    int k = 0;
    double p = 1.0;
    while (p > L) {
        k++;
        p *= ((double)rand() / RAND_MAX);
        if (k > 1000) break; /* safety */
    }
    return k - 1;
}

/* Append event log */
void log_event(int t, int approach_id, int passed) {
    if (event_log_len + 1 > event_log_capacity) {
        size_t newcap = (event_log_capacity == 0) ? 1024 : event_log_capacity * 2;
        EventLogEntry *tmp = realloc(event_log, newcap * sizeof(EventLogEntry));
        if (!tmp) {
            fprintf(stderr, "Failed to allocate event log memory\n");
            return;
        }
        event_log = tmp;
        event_log_capacity = newcap;
    }
    event_log[event_log_len].time_sec = t;
    event_log[event_log_len].approach_id = approach_id;
    event_log[event_log_len].vehicles_passed = passed;
    event_log_len++;
}

/* Reset metrics and queues for a fresh simulation run */
void reset_simulation_state() {
    event_log_len = 0;
    for (int i = 0; i < num_approaches; ++i) {
        approaches[i]->queue = 0;
        approaches[i]->cumulative_wait = 0.0;
        approaches[i]->total_arrived = 0;
        approaches[i]->total_served = 0;
        approaches[i]->cum_queue_length = 0.0;
    }
}

/* ---------- APPROACH MANAGEMENT (dynamic memory + pointer arrays) ---------- */

Approach *create_approach(int id, const char *name, int lanes, double arrival_rate, double service_rate) {
    Approach *ap = (Approach*) malloc(sizeof(Approach));
    if (!ap) return NULL;
    ap->id = id;
    strncpy(ap->name, name, MAX_NAME-1); ap->name[MAX_NAME-1] = '\0';
    ap->lanes = (lanes <= 0) ? 1 : ((lanes > MAX_LANES) ? MAX_LANES : lanes);
    ap->arrival_rate = arrival_rate;
    ap->service_rate = service_rate;
    ap->queue = 0;
    ap->cumulative_wait = 0.0;
    ap->total_arrived = 0;
    ap->total_served = 0;
    ap->cum_queue_length = 0.0;
    return ap;
}

void add_approach(Approach *ap) {
    Approach **tmp = realloc(approaches, (num_approaches + 1) * sizeof(Approach*));
    if (!tmp) {
        fprintf(stderr, "Failed to allocate approaches\n");
        return;
    }
    approaches = tmp;
    approaches[num_approaches] = ap;
    num_approaches++;
}

int find_approach_index_by_id(int id) {
    for (int i = 0; i < num_approaches; ++i) {
        if (approaches[i]->id == id) return i;
    }
    return -1;
}

void remove_approach_by_id(int id) {
    int idx = find_approach_index_by_id(id);
    if (idx == -1) {
        printf("Approach with id %d not found.\n", id);
        return;
    }
    free(approaches[idx]);
    for (int j = idx; j < num_approaches - 1; ++j) approaches[j] = approaches[j+1];
    num_approaches--;
    if (num_approaches == 0) {
        free(approaches); approaches = NULL;
    } else {
        Approach **tmp = realloc(approaches, num_approaches * sizeof(Approach*));
        if (tmp) approaches = tmp;
    }
    printf("Approach %d removed.\n", id);
}

void list_approaches() {
    if (num_approaches == 0) {
        printf("No approaches configured.\n");
        return;
    }
    printf("Configured approaches:\n");
    for (int i = 0; i < num_approaches; ++i) {
        Approach *a = approaches[i];
        printf("ID:%d Name:%s Lanes:%d Arr(rate)=%.3f/s Serv(rate)=%.3f/s Queue:%d\n",
               a->id, a->name, a->lanes, a->arrival_rate, a->service_rate, a->queue);
    }
}

/* ---------- LANE MAPPING (2D array example) ---------- */
/* For syllabus compliance: we create a simple 2D lane mapping array of size [num_approaches][max_lanes]
   storing lane indices or -1 for unused. Used only for representation / demonstration. */

int **make_lane_map() {
    if (num_approaches == 0) return NULL;
    int **map = malloc(num_approaches * sizeof(int*));
    for (int i = 0; i < num_approaches; ++i) {
        map[i] = malloc(MAX_LANES * sizeof(int));
        for (int l = 0; l < MAX_LANES; ++l) {
            if (l < approaches[i]->lanes) map[i][l] = l; else map[i][l] = -1;
        }
    }
    return map;
}

void free_lane_map(int **map) {
    if (!map) return;
    for (int i = 0; i < num_approaches; ++i) free(map[i]);
    free(map);
}

/* ---------- ADAPTIVE ALGORITHM ---------- */

/* Compute green allocations (in seconds) proportional to queue lengths,
   with min and max green bounds. Returns array of greens (malloc'd), caller frees. */
int *compute_adaptive_green_times(int min_green, int max_green, int cycle_time, int all_red) {
    int *greens = malloc(num_approaches * sizeof(int));
    if (!greens) return NULL;

    double total_queue = 0.0;
    for (int i = 0; i < num_approaches; ++i) total_queue += approaches[i]->queue;

    /* If total_queue == 0, assign equal small greens (min_green) rotated fairly */
    if (total_queue <= 0.0) {
        for (int i = 0; i < num_approaches; ++i) greens[i] = min_green;
    } else {
        double available = cycle_time - all_red;
        if (available < min_green * num_approaches) {
            /* Not enough time; set equal mins (can't satisfy constraints). */
            for (int i = 0; i < num_approaches; ++i) greens[i] = (int)floor(available / num_approaches);
            return greens;
        }
        /* initial proportional allocation */
        for (int i = 0; i < num_approaches; ++i) {
            double prop = approaches[i]->queue / total_queue;
            int g = (int)round(prop * available);
            if (g < min_green) g = min_green;
            if (g > max_green) g = max_green;
            greens[i] = g;
        }
        /* Adjust if sum exceeds available (scale down proportionally) */
        int sum = 0; for (int i = 0; i < num_approaches; ++i) sum += greens[i];
        if (sum > available) {
            double scale = available / (double)sum;
            for (int i = 0; i < num_approaches; ++i) {
                int g = (int)floor(greens[i] * scale);
                if (g < min_green) g = min_green;
                if (g > max_green) g = max_green;
                greens[i] = g;
            }
            /* final minor adjustments to exactly fill available */
            sum = 0; for (int i = 0; i < num_approaches; ++i) sum += greens[i];
            int idx = 0;
            while (sum < available) { greens[idx % num_approaches]++; sum++; idx++; }
            while (sum > available) { greens[idx % num_approaches]--; sum--; idx++; }
        } else if (sum < available) {
            /* distribute leftover seconds to approaches with largest queues */
            int leftover = (int)(available - sum);
            while (leftover > 0) {
                int best = -1; double bestq = -1;
                for (int i = 0; i < num_approaches; ++i) {
                    if (greens[i] < max_green && approaches[i]->queue > bestq) { bestq = approaches[i]->queue; best = i; }
                }
                if (best == -1) break;
                greens[best]++; leftover--;
            }
            /* if still leftover, distribute round-robin */
            int idx2 = 0;
            while (leftover > 0) { greens[idx2 % num_approaches]++; idx2++; leftover--; }
        }
    }
    return greens;
}

/* Fixed-time allocation: equal green per approach (bounded by min,max). */
int *compute_fixed_green_times(int min_green, int max_green, int cycle_time, int all_red) {
    int *greens = malloc(num_approaches * sizeof(int));
    if (!greens) return NULL;
    int available = cycle_time - all_red;
    int base = available / num_approaches;
    if (base < min_green) base = min_green;
    if (base > max_green) base = max_green;
    for (int i = 0; i < num_approaches; ++i) greens[i] = base;
    /* adjust to exactly available */
    int sum = base * num_approaches;
    int idx = 0;
    while (sum < available) { greens[idx % num_approaches]++; sum++; idx++; }
    while (sum > available) { greens[idx % num_approaches]--; sum--; idx++; }
    return greens;
}

/* ---------- SIMULATION CORE ---------- */

/* Simulate one cycle with given green times array (seconds per approach).
   Simulation runs for total_sim_time seconds (multiple cycles).
   For each second we:
   - generate arrivals for each approach based on arrival_rate
   - if approach has green in this second, serve at service_rate
   - update per-vehicle wait (approximate using queue levels)
*/
Metrics run_simulation(int cycle_time, int all_red, int min_green, int max_green,
                       int *green_times_per_cycle, int total_sim_time, int log_events) {
    reset_simulation_state();
    Metrics m = {0.0, 0, 0.0};
    if (num_approaches == 0 || total_sim_time <= 0) return m;

    int t = 0;
    while (t < total_sim_time) {
        /* For each approach: arrivals this second */
        for (int i = 0; i < num_approaches; ++i) {
            int arrivals = poisson_random(approaches[i]->arrival_rate); /* vehicles arriving in this 1 second */
            approaches[i]->queue += arrivals;
            approaches[i]->total_arrived += arrivals;
            /* accumulate queue for avg queue length */
            approaches[i]->cum_queue_length += approaches[i]->queue;
        }

        /* Determine which approach has green at this second according to cycle structure */
        int cycle_pos = t % cycle_time; /* 0 .. cycle_time-1 */
        int accum = 0;
        int active_approach = -1;
        for (int i = 0; i < num_approaches; ++i) {
            if (cycle_pos >= accum && cycle_pos < accum + green_times_per_cycle[i]) {
                active_approach = i;
                break;
            }
            accum += green_times_per_cycle[i];
        }
        /* If none active (e.g., in all-red region), active_approach stays -1 */
        /* Serve vehicles if active */
        if (active_approach >= 0) {
            Approach *ap = approaches[active_approach];
            /* vehicles that can be served in this second */
            int can_service = (int)floor(ap->service_rate + 1e-9); /* integer vehicles per second */
            if (can_service < 0) can_service = 0;
            if (ap->queue < can_service) can_service = ap->queue;
            ap->queue -= can_service;
            ap->total_served += can_service;
            m.throughput += can_service;
            /* approximate wait time: every vehicle in queue waited one more second this timestep */
            /* add per-second wait: number of vehicles waiting before service (we assume arrivals this sec do not wait whole sec)
               A practical approximation: add (queue length after arrivals and before service) to cumulative wait. */
            ap->cumulative_wait += (double)(ap->queue); /* each waiting vehicle accrues 1 second wait */
            /* log event for this second */
            if (log_events) log_event(t, ap->id, can_service);
        } else {
            /* all-red second: no service; but track waiting time for all queued vehicles */
            for (int i = 0; i < num_approaches; ++i) {
                approaches[i]->cumulative_wait += (double)(approaches[i]->queue);
            }
            if (log_events) log_event(t, -1, 0);
        }
        t++;
    }

    /* Compute average wait time per vehicle (sum of cumulative_waits / total served) */
    double total_wait = 0.0;
    long total_served = 0;
    double total_cum_queue = 0.0;
    for (int i = 0; i < num_approaches; ++i) {
        total_wait += approaches[i]->cumulative_wait;
        total_served += approaches[i]->total_served;
        total_cum_queue += approaches[i]->cum_queue_length;
    }
    m.throughput = total_served;
    m.avg_wait_time = (total_served > 0) ? (total_wait / (double)total_served) : 0.0;
    m.avg_queue_length = (total_sim_time > 0) ? (total_cum_queue / (double)(total_sim_time * num_approaches)) : 0.0;
    return m;
}

/* Wrapper: Run full simulation for either adaptive or fixed strategy,
   logs to file if requested, returns metrics. */
Metrics simulate_strategy(const char *strategy_name, int cycle_time, int all_red,
                          int min_green, int max_green, int total_sim_time,
                          int log_to_file, int adaptive) {
    if (num_approaches == 0) {
        printf("No approaches configured. Cannot simulate.\n");
        Metrics empty = {0.0, 0, 0.0};
        return empty;
    }

    /* We compute per-cycle green times that may change each cycle for adaptive.
       Implementation choice: for adaptive, recompute at start of each cycle using current queue snapshot.
       For fixed, compute once and reuse. */

    int *fixed_greens = NULL;
    if (!adaptive) {
        fixed_greens = compute_fixed_green_times(min_green, max_green, cycle_time, all_red);
    }

    /* prepare file logging if asked */
    FILE *f = NULL;
    if (log_to_file) {
        char fname[128];
        time_t now = time(NULL);
        struct tm *tm = localtime(&now);
        snprintf(fname, sizeof(fname), "event_log_%s_%04d%02d%02d_%02d%02d%02d.txt",
                 strategy_name,
                 tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday,
                 tm->tm_hour, tm->tm_min, tm->tm_sec);
        f = fopen(fname, "w");
        if (!f) {
            printf("Warning: cannot open file %s for writing; continuing without file logging.\n", fname);
            log_to_file = 0;
        } else {
            fprintf(f, "Event log for strategy: %s\n", strategy_name);
            fprintf(f, "Time(sec),ApproachID,VehiclesPassed\n");
        }
    }

    /* Run simulation cycle-by-cycle */
    reset_simulation_state();
    event_log_len = 0; /* reuse log buffer if any */
    int t = 0;
    while (t < total_sim_time) {
        /* At start of cycle compute greens */
        int *greens_this_cycle = NULL;
        if (adaptive) {
            /* recompute based on current queues */
            greens_this_cycle = compute_adaptive_green_times(min_green, max_green, cycle_time, all_red);
        } else {
            greens_this_cycle = fixed_greens; /* pointer to precomputed */
        }

        /* simulate one cycle worth of seconds */
        for (int sec = 0; sec < cycle_time && t < total_sim_time; ++sec, ++t) {
            /* arrivals */
            for (int i = 0; i < num_approaches; ++i) {
                int arrivals = poisson_random(approaches[i]->arrival_rate);
                approaches[i]->queue += arrivals;
                approaches[i]->total_arrived += arrivals;
                approaches[i]->cum_queue_length += approaches[i]->queue;
            }
            /* who has green at this second */
            int accum = 0;
            int active_index = -1;
            for (int i = 0; i < num_approaches; ++i) {
                if (sec >= accum && sec < accum + greens_this_cycle[i]) { active_index = i; break; }
                accum += greens_this_cycle[i];
            }
            if (active_index >= 0) {
                Approach *ap = approaches[active_index];
                int can_service = (int)floor(ap->service_rate + 1e-9);
                if (can_service < 0) can_service = 0;
                if (ap->queue < can_service) can_service = ap->queue;
                ap->queue -= can_service;
                ap->total_served += can_service;
                /* waiting vehicles (after serving this second) accrue 1 second wait each */
                ap->cumulative_wait += (double)(ap->queue);
                if (log_to_file && f) fprintf(f, "%d,%d,%d\n", t, ap->id, can_service);
                if (!log_to_file) log_event(t, ap->id, can_service);
            } else {
                /* all-red second */
                for (int i = 0; i < num_approaches; ++i)
                    approaches[i]->cumulative_wait += (double)(approaches[i]->queue);
                if (log_to_file && f) fprintf(f, "%d,%d,%d\n", t, -1, 0);
                if (!log_to_file) log_event(t, -1, 0);
            }
        }
        if (adaptive && greens_this_cycle) free(greens_this_cycle);
    }

    if (fixed_greens) free(fixed_greens);
    if (f) fclose(f);

    /* compute metrics */
    Metrics res = {0.0, 0, 0.0};
    long total_served = 0; double total_wait = 0.0, total_cum_queue = 0.0;
    for (int i = 0; i < num_approaches; ++i) {
        total_served += approaches[i]->total_served;
        total_wait += approaches[i]->cumulative_wait;
        total_cum_queue += approaches[i]->cum_queue_length;
    }
    res.throughput = total_served;
    res.avg_wait_time = (total_served > 0) ? (total_wait / (double)total_served) : 0.0;
    res.avg_queue_length = (total_sim_time > 0) ? (total_cum_queue / (double)(total_sim_time * num_approaches)) : 0.0;
    return res;
}

/* ---------- USER INTERFACE ---------- */

void configure_default_sample() {
    /* sample 4-approach intersection (N, E, S, W) */
    for (int i = 0; i < num_approaches; ++i) {
        free(approaches[i]);
    }
    free(approaches);
    approaches = NULL;
    num_approaches = 0;

    add_approach(create_approach(1, "North", 2, 0.5, 2.0));
    add_approach(create_approach(2, "East", 2, 0.4, 2.0));
    add_approach(create_approach(3, "South", 2, 0.6, 2.0));
    add_approach(create_approach(4, "West", 2, 0.3, 2.0));
    printf("Sample 4-approach intersection configured.\n");
}

void interactive_add_approach() {
    int id, lanes;
    char name[MAX_NAME];
    double arr, serv;
    printf("Enter approach id (integer): ");
    scanf("%d", &id);
    printf("Enter name/direction (single word): ");
    scanf("%s", name);
    printf("Enter number of lanes (1-%d): ", MAX_LANES);
    scanf("%d", &lanes);
    printf("Enter arrival rate (vehicles per second, e.g., 0.5): ");
    scanf("%lf", &arr);
    printf("Enter service rate when green (veh/s, e.g., 2.0): ");
    scanf("%lf", &serv);
    Approach *ap = create_approach(id, name, lanes, arr, serv);
    add_approach(ap);
    printf("Approach added.\n");
}

void interactive_edit_approach() {
    int id;
    printf("Enter approach id to edit: ");
    scanf("%d", &id);
    int idx = find_approach_index_by_id(id);
    if (idx == -1) { printf("Not found.\n"); return; }
    Approach *ap = approaches[idx];
    printf("Editing approach %d (%s). Leave a value as -1 to keep unchanged.\n", ap->id, ap->name);
    printf("Enter new arrival rate (current %.3f): ", ap->arrival_rate);
    double ar; scanf("%lf", &ar); if (ar >= 0) ap->arrival_rate = ar;
    printf("Enter new service rate (current %.3f): ", ap->service_rate);
    double sr; scanf("%lf", &sr); if (sr >= 0) ap->service_rate = sr;
    printf("Enter new lanes (current %d): ", ap->lanes);
    int ln; scanf("%d", &ln); if (ln > 0 && ln <= MAX_LANES) ap->lanes = ln;
    printf("Updated.\n");
}

void save_event_log_to_file() {
    if (event_log_len == 0) { printf("No events to save.\n"); return; }
    char fname[128];
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    snprintf(fname, sizeof(fname), "event_log_run_%04d%02d%02d_%02d%02d%02d.csv",
             tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec);
    FILE *f = fopen(fname, "w");
    if (!f) { printf("Failed to open file for writing.\n"); return; }
    fprintf(f, "time_sec,approach_id,vehicles_passed\n");
    for (size_t i = 0; i < event_log_len; ++i) {
        fprintf(f, "%d,%d,%d\n", event_log[i].time_sec, event_log[i].approach_id, event_log[i].vehicles_passed);
    }
    fclose(f);
    printf("Event log saved to %s\n", fname);
}

/* Comparison run: runs adaptive and fixed, prints comparison */
void run_and_compare(int cycle_time, int all_red, int min_green, int max_green, int sim_seconds) {
    printf("Running ADAPTIVE simulation for %d seconds...\n", sim_seconds);
    Metrics m_ad = simulate_strategy("adaptive", cycle_time, all_red, min_green, max_green, sim_seconds, 0, 1);
    printf("Adaptive -> Throughput: %ld vehicles, Avg wait: %.3f s, Avg queue: %.3f\n",
           m_ad.throughput, m_ad.avg_wait_time, m_ad.avg_queue_length);

    /* Save state of approaches so we can reset before fixed run */
    /* We'll snapshot current approach configuration values (arrival rates, etc.) */
    /* Resetting internal queues and counters is handled inside simulate_strategy */
    printf("Running FIXED-TIME simulation for %d seconds...\n", sim_seconds);
    Metrics m_fx = simulate_strategy("fixed", cycle_time, all_red, min_green, max_green, sim_seconds, 0, 0);
    printf("Fixed -> Throughput: %ld vehicles, Avg wait: %.3f s, Avg queue: %.3f\n",
           m_fx.throughput, m_fx.avg_wait_time, m_fx.avg_queue_length);

    printf("\nComparison (Adaptive vs Fixed):\n");
    printf("Metric\t\tAdaptive\tFixed\t\tBetter\n");
    printf("Throughput\t%ld\t\t%ld\t\t%s\n",
           m_ad.throughput, m_fx.throughput,
           (m_ad.throughput > m_fx.throughput) ? "Adaptive" : ((m_fx.throughput > m_ad.throughput) ? "Fixed" : "Tie"));
    printf("Avg Wait (s)\t%.3f\t\t%.3f\t\t%s\n",
           m_ad.avg_wait_time, m_fx.avg_wait_time,
           (m_ad.avg_wait_time < m_fx.avg_wait_time) ? "Adaptive" : ((m_fx.avg_wait_time < m_ad.avg_wait_time) ? "Fixed" : "Tie"));
    printf("Avg Queue\t%.3f\t\t%.3f\t\t%s\n",
           m_ad.avg_queue_length, m_fx.avg_queue_length,
           (m_ad.avg_queue_length < m_fx.avg_queue_length) ? "Adaptive" : ((m_fx.avg_queue_length < m_ad.avg_queue_length) ? "Fixed" : "Tie"));
}

/* ---------- MAIN MENU ---------- */

int main() {
    seed_rng();
    printf("Adaptive Traffic Intersection Controller (Sensor Driven) — Console Simulation\n");
    printf("Syllabus topics: control, loops, arrays, 2D arrays, structures, pointers, dynamic memory\n\n");

    int cycle_time = DEFAULT_CYCLE;
    int all_red = DEFAULT_ALL_RED;
    int min_green = DEFAULT_MIN_GREEN;
    int max_green = DEFAULT_MAX_GREEN;

    int choice;
    do {
        printf("\n==== MAIN MENU ====\n");
        printf("1) Configure sample intersection (4 approaches)\n");
        printf("2) Add approach\n");
        printf("3) Remove approach\n");
        printf("4) Edit approach\n");
        printf("5) List approaches\n");
        printf("6) Show 2D lane map (demo)\n");
        printf("7) Run adaptive simulation (single run)\n");
        printf("8) Run fixed-time simulation (single run)\n");
        printf("9) Compare adaptive vs fixed\n");
        printf("10) Save last in-memory event log to file\n");
        printf("11) Change timing parameters (cycle/all-red/min/max)\n");
        printf("0) Exit\n");
        printf("Choose: ");
        scanf("%d", &choice);

        switch (choice) {
            case 1:
                configure_default_sample();
                break;
            case 2:
                interactive_add_approach();
                break;
            case 3: {
                int id; printf("Enter approach id to remove: "); scanf("%d", &id);
                remove_approach_by_id(id);
                break;
            }
            case 4:
                interactive_edit_approach();
                break;
            case 5:
                list_approaches();
                break;
            case 6: {
                int **map = make_lane_map();
                if (!map) { printf("No map (no approaches).\n"); break; }
                printf("Lane map (rows=approaches, cols=lanes index or -1):\n");
                for (int i = 0; i < num_approaches; ++i) {
                    printf("%s: ", approaches[i]->name);
                    for (int l = 0; l < MAX_LANES; ++l) printf("%d ", map[i][l]);
                    printf("\n");
                }
                free_lane_map(map);
                break;
            }
            case 7: {
                int sim_s; printf("Total simulation time (seconds)? (e.g., 600): "); scanf("%d", &sim_s);
                event_log_len = 0;
                Metrics m = simulate_strategy("adaptive_run", cycle_time, all_red, min_green, max_green, sim_s, 0, 1);
                printf("Adaptive run complete. Throughput:%ld Avg wait:%.3f s Avg queue:%.3f\n",
                       m.throughput, m.avg_wait_time, m.avg_queue_length);
                break;
            }
            case 8: {
                int sim_s; printf("Total simulation time (seconds)? (e.g., 600): "); scanf("%d", &sim_s);
                event_log_len = 0;
                Metrics m = simulate_strategy("fixed_run", cycle_time, all_red, min_green, max_green, sim_s, 0, 0);
                printf("Fixed run complete. Throughput:%ld Avg wait:%.3f s Avg queue:%.3f\n",
                       m.throughput, m.avg_wait_time, m.avg_queue_length);
                break;
            }
            case 9: {
                int sim_s; printf("Comparison simulation time (seconds)? (e.g., 600): "); scanf("%d", &sim_s);
                run_and_compare(cycle_time, all_red, min_green, max_green, sim_s);
                break;
            }
            case 10:
                save_event_log_to_file();
                break;
            case 11: {
                printf("Current: cycle=%d all_red=%d min_green=%d max_green=%d\n", cycle_time, all_red, min_green, max_green);
                printf("Enter cycle time (s): "); scanf("%d", &cycle_time);
                printf("Enter all-red (s): "); scanf("%d", &all_red);
                printf("Enter min green (s): "); scanf("%d", &min_green);
                printf("Enter max green (s): "); scanf("%d", &max_green);
                break;
            }
            case 0:
                printf("Exiting. Freeing memory.\n");
                break;
            default:
                printf("Invalid choice.\n");
        }
    } while (choice != 0);

    /* cleanup */
    for (int i = 0; i < num_approaches; ++i) free(approaches[i]);
    free(approaches);
    free(event_log);

    return 0;
}
