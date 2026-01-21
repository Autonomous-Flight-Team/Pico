

// structs

struct linear_vector
{
    double x{0}, y{0}, z{0};
};

// PID state holder

struct pidHistory
{
    double x_error{0}, y_error{0}, z_error{0}; // holds Previous error
    double x_integral{0}, y_integral{0}, z_integral{0};
    double x_derivative{0}, y_derivative{0}, z_derivative{0};
};

// motor torque struct. holds info about each motor, may be torque or voltage depending on when used
struct motor_vector
{
    double front_left_motor{0}, back_left_motor{0}, front_right_motor{0}, back_right_motor{0};
};

//Holds all values needed for attitude
struct attitude_vector
{
    double roll{0};
    double pitch{0};
    double yaw{0};
    double thrust{0};
};