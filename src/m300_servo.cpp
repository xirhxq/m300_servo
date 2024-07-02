#include "FlightControl.hpp"
#include "DataLogger.hpp"

using namespace dji_osdk_ros;
using namespace std;

class TASK {

private:

    FLIGHT_CONTROL fc;

    typedef enum { TAKEOFF, ASCEND, SEARCH, TRACK, HOLD, BACK, LAND, END } ControlState;
    ControlState task_state;
    double task_begin_time, task_time;
    Point desired_point;
    double expected_height = 1.0;

    double tic, toc;

    ros::Rate rate;

    ros::Subscriber searchPointSub;

public:

    TASK(string name, bool ON_GROUND, string start_state, ros::NodeHandle nh_): 
        fc(name, nh_), rate(50){

        searchPointSub = nh_.subscribe(name + "/searchPoint", 1, &TASK::searchPointCallback, this);

        for (int i = 0; i < 100; i++) {
            ros::spinOnce();
            rate.sleep();
        }

        fc.yaw_offset = fc.current_euler_angle.z;
        printf("Yaw offset: %.2lf\n", fc.yaw_offset * RAD2DEG_COE);
        for (int i = 0; i < 100; i++) {
            ros::spinOnce();
            rate.sleep();
        }
        setValue(fc.position_offset, fc.current_pos_raw);
        printf("Position offset: %s\n", outputStr(fc.position_offset).c_str());

        printf("Use supersonic wave for height, now_height: %.2lf\n", fc.current_pos_raw.z);

        printf("Waiting for command to take off...\n");
        sleep(3);
        if (!ON_GROUND) {
            fc.obtain_control();
            fc.monitoredTakeoff();
        }


        printf("Start Control State Machine...\n");

        toStepTakeoff();
        if (start_state == "takeoff") {
            toStepTakeoff();
        }
        else if (start_state == "land") {
            toStepLand();
        }
        else if (start_state == "hold") {
            toStepHold();
        }

    }

    void searchPointCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    }

    void toStepTakeoff(){
        tic = fc.get_time_now();
        task_state = TAKEOFF;
        task_begin_time = fc.get_time_now();
    }

    void toStepHold(){
        tic = fc.get_time_now();
        task_state = HOLD;
    }

    void toStepLand(){
        tic = fc.get_time_now();
        task_state = LAND;
    }

    void toStepBack(){
        tic = fc.get_time_now();
        task_state = BACK;
    }

    void toStepEnd() {
        tic = fc.get_time_now();
        task_state = END;
    }

    void StepTakeoff() {
        printf("###----StepTakeoff----###\n");
        printf("Expected height @ %.2lf\n", expected_height);
        fc.M210_position_yaw_ctrl(0, 0, expected_height, fc.yaw_offset);
        if (toc - tic >= 15.0){
            // printf("Arrive expected height @ %.2lf\n", expected_height);
            toStepHold();
        }
    }

    void StepHold() {
        printf("###----StepHold----###\n");
        double hold_time = 20.0;
        auto expected_point = newPoint(0, 0, expected_height);
        printf("Hold %.2lf\n", toc - tic);
        printf("ExpectedPoint: %s\n", outputStr(expected_point).c_str());
        fc.M210_velocity_position_yaw_ctrl(0, 0, expected_height, fc.yaw_offset);
        // fc.M210_position_yaw_ctrl(0, 0, expected_height, fc.yaw_offset);
        // fc.UAV_Control_to_Point_with_yaw(expected_point, fc.yaw_offset);
        if (toc - tic >= 20.0){
            toStepBack();
        }
    }

    void StepBack() {
        printf("###----StepBack----###\n");
        double hold_time = 5.0;
        auto expected_point = newPoint(0, 0, 2.5);
        printf("ExpectedPoint: %s\n", outputStr(expected_point).c_str());
        // fc.UAV_Control_to_Point_with_yaw(expected_point, fc.yaw_offset);
        fc.M210_position_yaw_rate_ctrl(0, 0, 2.5, 0);
        if (toc - tic >= 20.0){
              toStepLand();
        }
    }

    void StepLand() {
        printf("###----StepLand----###\n");
        printf("Landing...\n");
        fc.takeoff_land(dji_osdk_ros::DroneTaskControl::Request::TASK_LAND);
        if (nearlyIs(fc.current_pos_raw.z, 0.1, 0.2)) {
            toStepEnd();
        }
        // task_state = BACK;
    }

    void ControlStateMachine() {
        switch (task_state) {
            case TAKEOFF: {
                StepTakeoff();
                break;
            }
            case HOLD: {
                StepHold();
                break;
            }
            case LAND: {
                StepLand();
                break;
            }
            case BACK: {
                StepBack();
                break;
            }
            default: {
                break;
            }
        }
    }

    void spin(){

        while (ros::ok()) {
            // std::cout << "\033c" << std::flush;
            toc = fc.get_time_now();
            task_time = fc.get_time_now() - task_begin_time;
            printf("-----------\n");
            printf("Time: %lf\n", task_time);
            printf("M210(State: %d) @ %s\n", task_state, outputStr(fc.current_pos_raw).c_str());
            printf("Gimbal %s\n", outputStr(fc.current_gimbal_angle).c_str());
            printf(
                "Attitude (R%.2lf, P%.2lf, Y%.2lf) / deg\n", 
                fc.current_euler_angle.x * RAD2DEG_COE,
                fc.current_euler_angle.y * RAD2DEG_COE,
                fc.current_euler_angle.z * RAD2DEG_COE
            );                     
            printf("State time: %.2lf\n", toc - tic);
            if (fc.EMERGENCY) {
                fc.M210_hold_ctrl();
                printf("!!!!!!!!!!!!EMERGENCY!!!!!!!!!!!!\n");
            }
            else {
                ControlStateMachine();
                if (task_state == END) {
                    break;
                }
            }
            
            ros::spinOnce();
            rate.sleep();
        }
    }

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "suav", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    if (argc <= 2) {
        ROS_ERROR("Required more than 2 parameters, quitting...");
        return 0;
    }

    bool ON_GROUND = true;
    if (argc > 2) {
        auto on_ground = std::string(argv[2]);
        if (on_ground == "false" || on_ground == "takeoff") {
            ON_GROUND = false;
            ROS_WARN("WILL TAKE OFF!!!!");
        }
        else {
            ROS_WARN("ON GROUND TEST!!!");
        }
    }
    
	string uav_name = "none";
	if (argc > 1) {
		uav_name = std::string(argv[1]);
	}
    while (uav_name == "none"){
        ROS_ERROR("Invalid vehicle name: %s", uav_name.c_str());
    }
	printf("Vehicle name: %s\n", uav_name.c_str());


    std::string start_state = "";
    if (argc > 3) {
        // get the start state and set the task_state
        start_state = std::string(argv[3]);
    }


    TASK t(uav_name, ON_GROUND, start_state, nh);

    

    t.spin();
    return 0;
}
