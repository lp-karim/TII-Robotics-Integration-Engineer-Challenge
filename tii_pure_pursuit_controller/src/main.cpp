int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_controller");
    ros::NodeHandle nh;

    PurePursuitController controller(nh);

    ros::spin()
    return 0;
}