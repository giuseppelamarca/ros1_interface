#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define N_CELLS 6

struct header{
    uint32_t seq;
    int32_t sec;
    int32_t nsec;
    char frame_id[100];
};

struct plc{
    long int mtype;
    float v_right;
    float v_left;
    int8_t mode;
    int32_t current_right;
    int32_t current_left;
    bool brake_active;
    bool safety;
    bool error_status;
    char* error_description;
};

struct twist{
    long int mtype;
    struct lin{
    double x;
    double y;
    double z;
    }linear;
    struct ang{
    double x;
    double y;
    double z;
    }angular;
};

struct pose{
    long int mtype;
    struct pos{
    double x;
    double y;
    double z;
    }position;
    struct ori{
    double x;
    double y;
    double z;
    double w;
    }orientation;
};


struct cmd_vel{
    long int mtype;
    struct header h_;
    struct twist twist_;
};

struct odom{
    long int mtype;
    struct header header_;
    char child_frame_id[100];
    struct pose pose_;
    double pose_covariance[36];
    struct twist twist_;
    double twist_covariance[36];
    long int t;
};

struct laser_scan{
    long int mtype;
    struct header header_;
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    float ranges[504];
    float intensities[504];
    long int t;
};

struct battery{
    long int mtype;
    struct header header_;
    float voltage;
    float current;
    float charge;
    float capacity;
    float design_capacity;
    float percentage;
    uint8_t power_supply_status;
    uint8_t power_supply_health;
    uint8_t power_supply_technology;
    bool present;
    float cell_voltage[N_CELLS];
    char location[100];
    char serial_number[100];
    long int t;
};

struct diagnostics_agg{
    long int mtype;
    struct header header_;
    uint8_t level;
    char name[100];
    char message[1000];
    char hardware_id[100];
    long int t;
};

