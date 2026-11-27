// Synthetic calibrations and a pseudo-terminal only. Never opens a real port.
#include "../src/arm_system.cpp"
#include <pty.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <iostream>
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
void require(bool ok, const char* why) { if(!ok) throw std::runtime_error(why); }
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  int master,slave;char port[128];
  if(openpty(&master,&slave,port,nullptr,nullptr)) return 2;
  std::atomic<bool> running{true};std::atomic<int> moves{0};
  std::array<int,6> raw{{1600,1500,1500,1500,1500,1500}};
  std::mutex mutex;
  std::thread device([&] {
    std::string frame;
    while(running) {
      pollfd p{master,POLLIN,0};if(::poll(&p,1,20)<=0)continue;
      char c;if(::read(master,&c,1)!=1)continue;
      if(c=='#')frame.clear();frame+=c;
      if(c!='!')continue;
      if(frame.size()>=9) {
        int id=std::stoi(frame.substr(1,3));
        if(id>=0&&id<6) {
          std::lock_guard<std::mutex> guard(mutex);
          if(frame.substr(4)=="PRAD!") {
            std::ostringstream reply;reply<<arm_hardware::prefix(id)<<std::setw(4)<<std::setfill('0')<<raw[id]<<'!';
            auto s=reply.str();::write(master,s.data(),s.size());
          } else if(frame.size()==15 && frame[9]=='T') {
            raw[id]=std::stoi(frame.substr(5,4));++moves;
          }
        }
      }
      frame.clear();
    }
  });
  int code=0;
  try {
    const std::vector<std::string> names{"base_joint","shoulder_joint","elbow_joint","wrist_pitch_joint","wrist_rotation_joint","gripper_drive_joint","left_rocker_joint","left_finger_joint","right_drive_joint","right_rocker_joint","right_finger_joint"};
    const std::array<double,5> factors{{.993405221421939,-1.0164590876196737,-1.,-.993405221421939,1.0164590876196737}};
    hardware_interface::HardwareInfo info;info.name="TestCad";info.type="system";
    info.hardware_parameters={{"model","cad"},{"port",port},{"enable_motion","true"},{"limits_verified","true"},{"enable_step_check","false"}};
    for(size_t i=0;i<names.size();++i) {
      hardware_interface::ComponentInfo joint;joint.name=names[i];joint.type="joint";
      if(i<6)joint.parameters={{"raw_zero",i==0?"1600":(i==5?"1400":"1500")},{"radians_per_raw",i==1?"-0.002":"0.002"},{"raw_min","500"},{"raw_max","2500"}};
      hardware_interface::InterfaceInfo pos;pos.name="position";joint.state_interfaces.push_back(pos);
      hardware_interface::InterfaceInfo vel;vel.name="velocity";joint.state_interfaces.push_back(vel);
      if(i<6)joint.command_interfaces.push_back(pos);
      info.joints.push_back(joint);
      if(i>=6)info.mimic_joints.push_back({i,5,factors[i-6],0.});
    }
    const auto original_info=info;
    arm_hardware::ArmSystem driver;
    require(driver.on_init(info)==CallbackReturn::SUCCESS,"CAD init");
    rclcpp_lifecycle::State state;
    require(driver.on_configure(state)==CallbackReturn::SUCCESS,"CAD configure");
    require(driver.on_activate(state)==CallbackReturn::SUCCESS,"CAD activate");
    auto states=driver.export_state_interfaces();auto commands=driver.export_command_interfaces();
    require(states.size()==22&&commands.size()==6,"Interface count");
    require(moves==0,"Activation must not move");
    require(std::abs(states[10].get_value()-.2)<1e-9,"raw -> CAD gripper radians");
    commands[5].set_value(.24);commands[1].set_value(-.04);
    require(driver.write(rclcpp::Time(0),rclcpp::Duration(0,20000000))==return_type::OK,"CAD write");
    for(int i=0;i<6;++i)require(driver.read(rclcpp::Time(0),rclcpp::Duration(0,20000000))==return_type::OK,"CAD read");
    {std::lock_guard<std::mutex> guard(mutex);require(raw[5]==1520&&raw[1]==1520,"Correct IDs, angle sign and serial raw");}
    for(size_t i=0;i<5;++i)require(std::abs(states[2*(i+6)].get_value()-.24*factors[i])<1e-9,"Mimic feedback");
    const int sent=moves;
    commands[5].set_value(99.);
    require(driver.write(rclcpp::Time(0),rclcpp::Duration(0,20000000))==return_type::ERROR,"Reject out-of-range");
    require(moves==sent,"No out-of-range packet");
    driver.on_cleanup(state);
    // A missing mapping must fail before opening a port.
    info.joints[5].parameters.erase("raw_zero");arm_hardware::ArmSystem invalid;
    require(invalid.on_init(info)==CallbackReturn::ERROR,"Missing calibration rejected");
    // Legacy metre-based gripper retains the original conversion and shape.
    info.hardware_parameters["model"]="legacy";info.hardware_parameters["enable_motion"]="false";
    info.joints.resize(7);info.joints[5].name="left_finger_joint";info.joints[6].name="right_finger_joint";
    info.mimic_joints={{6,5,-1.,0.}};
    {std::lock_guard<std::mutex> guard(mutex);raw[5]=1500;}
    arm_hardware::ArmSystem legacy;
    require(legacy.on_init(info)==CallbackReturn::SUCCESS,"Legacy init");
    require(legacy.on_configure(state)==CallbackReturn::SUCCESS,"Legacy configure");
    require(legacy.on_activate(state)==CallbackReturn::SUCCESS,"Legacy activate");
    auto oldstates=legacy.export_state_interfaces();auto oldcommands=legacy.export_command_interfaces();
    require(oldstates.size()==14&&std::abs(oldstates[10].get_value()-.0175)<1e-9,"Legacy metres unchanged");
    oldcommands[5].set_value(.016);
    require(legacy.write(rclcpp::Time(0),rclcpp::Duration(0,20000000))==return_type::ERROR,"Read-only rejects movement");
    require(moves==sent,"Read-only did not send movement");legacy.on_cleanup(state);
    info=original_info;info.hardware_parameters["limits_verified"]="false";
    info.hardware_parameters["commissioning_base_test"]="true";
    arm_hardware::ArmSystem trial;
    require(trial.on_init(info)==CallbackReturn::SUCCESS,"Commissioning init without claiming limits verified");
    require(trial.on_configure(state)==CallbackReturn::SUCCESS,"Trial configure");
    require(trial.on_activate(state)==CallbackReturn::SUCCESS,"Trial activate");
    auto tc=trial.export_command_interfaces();auto ts=trial.export_state_interfaces();
    tc[0].set_value(ts[0].get_value()+.02);
    require(trial.write(rclcpp::Time(0),rclcpp::Duration(0,20000000))==return_type::OK,"Small base move accepted");
    for(int i=0;i<6;++i)trial.read(rclcpp::Time(0),rclcpp::Duration(0,20000000));
    const int trial_sent=moves;
    tc[1].set_value(ts[2].get_value()+.02);
    require(trial.write(rclcpp::Time(0),rclcpp::Duration(0,20000000))==return_type::ERROR,"Trial blocks other motors");
    tc[1].set_value(ts[2].get_value());tc[0].set_value(.2);
    require(trial.write(rclcpp::Time(0),rclcpp::Duration(0,20000000))==return_type::ERROR,"Trial blocks large base move");
    require(moves==trial_sent,"Blocked trial has no packets");trial.on_cleanup(state);
    std::cout<<"PASS: CAD conversion, IDs, mimic feedback, no startup movement, bounds, missing calibration and legacy read-only\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';code=1;}
  running=false;device.join();close(slave);close(master);rclcpp::shutdown();return code;
}
