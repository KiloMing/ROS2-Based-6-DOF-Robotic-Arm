#include <array>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <memory>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_hardware {
namespace {
using Clock = std::chrono::steady_clock;
constexpr double rad_per_raw = 0.00235619449;
struct Cal { const char * name; int zero, sign, low, high; };
constexpr std::array<Cal, 6> cal{{
  {"base_joint", 1600, 1, 500, 2500},
  {"shoulder_joint", 1500, -1, 700, 2300},
  {"elbow_joint", 1500, 1, 500, 2500},
  {"wrist_pitch_joint", 1500, -1, 500, 2500},
  {"wrist_rotation_joint", 1500, 1, 500, 2500},
  {"left_finger_joint", 0, 0, 1100, 1770}
}};
constexpr std::array<std::pair<int, double>, 8> grip{{
  {1100,.060}, {1200,.055}, {1300,.050}, {1400,.045},
  {1500,.035}, {1600,.020}, {1700,.005}, {1770,.000}
}};
double finger_from_raw(int r) {
  if (r < 1100 || r > 1770) throw std::runtime_error("Gripper feedback out of range");
  for (size_t i=1; i<grip.size(); ++i) {
    const auto a=grip[i-1], b=grip[i];
    if (r<=b.first) return .5*(a.second+(r-a.first)*(b.second-a.second)/(b.first-a.first));
  }
  return 0.0;
}
int finger_to_raw(double q) {
  const double opening=2*q;
  if (!std::isfinite(q) || opening<0 || opening>.060) throw std::runtime_error("Invalid gripper goal");
  for (size_t i=1; i<grip.size(); ++i) {
    const auto a=grip[i-1], b=grip[i];
    if (opening>=b.second)
      return int(std::lround(a.first+(opening-a.second)*(b.first-a.first)/(b.second-a.second)));
  }
  return 1770;
}
std::string prefix(int id) {
  std::ostringstream o;
  o << '#' << std::setw(3) << std::setfill('0') << id << 'P';
  return o.str();
}
class Serial {
public:
  ~Serial() { close_port(); }
  void close_port() { if (fd_>=0) {::close(fd_);fd_=-1;} }
  void open_port(const std::string& name) {
    close_port();
    fd_=::open(name.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    if(fd_<0) throw std::runtime_error("Cannot open serial device: "+name);
    termios t{};
    if(::tcgetattr(fd_,&t)) {close_port();throw std::runtime_error("tcgetattr failed");}
    ::cfmakeraw(&t);
    ::cfsetispeed(&t,B115200);::cfsetospeed(&t,B115200);
    t.c_cflag|=CLOCAL|CREAD;
    t.c_cflag&=~(CRTSCTS|PARENB|CSTOPB);
    t.c_cflag=(t.c_cflag&~CSIZE)|CS8;
    t.c_cc[VMIN]=0;t.c_cc[VTIME]=0;
    if(::tcsetattr(fd_,TCSANOW,&t)) {close_port();throw std::runtime_error("tcsetattr failed");}
    ::tcflush(fd_,TCIOFLUSH);
  }
  void send(const std::string& s) {
    if(fd_<0)throw std::runtime_error("Serial closed");
    auto deadline=Clock::now()+std::chrono::milliseconds(80);
    size_t pos=0;
    while(pos<s.size()) {
      auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()).count();
      if(ms<=0)throw std::runtime_error("Serial write timeout");
      pollfd p{fd_,POLLOUT,0};
      int rc=::poll(&p,1,int(ms));
      if(rc<0 && errno==EINTR)continue;
      if(rc<=0 || (p.revents&(POLLHUP|POLLERR|POLLNVAL)))throw std::runtime_error("Serial write poll error");
      ssize_t n=::write(fd_,s.data()+pos,s.size()-pos);
      if(n<0 && (errno==EAGAIN||errno==EINTR))continue;
      if(n<=0)throw std::runtime_error("Serial write error");
      pos+=size_t(n);
    }
  }
  int position(int id,int timeout_ms) {
    if(fd_<0)throw std::runtime_error("Serial closed");
    const auto tag=prefix(id);
    ::tcflush(fd_,TCIFLUSH);
    send(tag+"RAD!");
    auto deadline=Clock::now()+std::chrono::milliseconds(timeout_ms);
    std::string frame;
    while(Clock::now()<deadline) {
      auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()).count();
      pollfd p{fd_,POLLIN,0};
      int rc=::poll(&p,1,int(ms>0?ms:1));
      if(rc<0&&errno==EINTR)continue;
      if(rc<=0)continue;
      if(p.revents&(POLLHUP|POLLERR|POLLNVAL))throw std::runtime_error("Serial read poll error");
      char data[128];
      ssize_t n=::read(fd_,data,sizeof(data));
      if(n<0&&(errno==EINTR||errno==EAGAIN))continue;
      if(n<0)throw std::runtime_error("Serial read error");
      for(ssize_t i=0;i<n;++i) {
        if(data[i]=='#'){frame="#";continue;}
        if(frame.empty())continue;
        if(frame.size()>32){frame.clear();continue;}
        frame+=data[i];
        if(data[i]!='!')continue;
        bool good=frame.size()==tag.size()+5&&frame.compare(0,tag.size(),tag)==0;
        if(good)for(size_t k=tag.size();k<tag.size()+4;++k)
          if(frame[k]<'0'||frame[k]>'9')good=false;
        if(good)return std::stoi(frame.substr(tag.size(),4));
        frame.clear();  // Ignore echoes; only #NNNPdddd! is accepted.
      }
    }
    throw std::runtime_error("Missing PRAD response for ID "+std::to_string(id));
  }
  void move(int id,int raw,int ms) {
    std::ostringstream o;
    o<<prefix(id)<<std::setw(4)<<std::setfill('0')<<raw
      <<'T'<<std::setw(4)<<std::setfill('0')<<ms<<'!';
    send(o.str());
  }
private:
  int fd_{-1};
};
}  // namespace

class ArmSystem : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override {
    if(hardware_interface::SystemInterface::on_init(info)!=hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;
    try {
      if(info_.joints.size()!=7)throw std::runtime_error("Expected 6 motors + 1 mimic joint");
      for(size_t i=0;i<7;++i) {
        const std::string name=i==6?"right_finger_joint":cal[i].name;
        if(info_.joints[i].name!=name)throw std::runtime_error("Joint order mismatch: "+info_.joints[i].name);
      }
      const auto& p=info_.hardware_parameters;
      auto get=[&](const std::string& k,const std::string& d){
        auto it=p.find(k);return it==p.end()?d:it->second;
      };
      port_=get("port","/dev/ttyUSB0");
      if(get("baud","115200")!="115200")throw std::runtime_error("Only 115200 baud supported");
      timeout_=std::stoi(get("timeout_ms","40"));
      max_age_=std::stoi(get("max_feedback_age_ms","1200"));
      step_=std::stoi(get("max_step_raw","8"));
      tracking_=std::stoi(get("max_tracking_raw","80"));
      duration_=std::stoi(get("move_time_ms","180"));
      motion_=get("enable_motion","false")=="true";
      checked_=get("limits_verified","false")=="true";
      if(timeout_<10||timeout_>500||max_age_<500||step_<1||step_>30||
         tracking_<step_||tracking_>200||duration_<50||duration_>1000)
        throw std::runtime_error("Invalid timing/safety parameter");
      if(motion_&&!checked_)throw std::runtime_error("Motion requires limits_verified=true");
      RCLCPP_WARN(rclcpp::get_logger("arm_hardware"),
        "YH-K32 driver: %s; historical raw ranges are NOT certified physical limits",
        motion_?"MOTION ENABLED":"READ ONLY");
      return hardware_interface::CallbackReturn::SUCCESS;
    }catch(const std::exception& e){
      RCLCPP_ERROR(rclcpp::get_logger("arm_hardware"),"init: %s",e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> out;
    for(size_t i=0;i<7;++i){
      out.emplace_back(info_.joints[i].name,hardware_interface::HW_IF_POSITION,&q_[i]);
      out.emplace_back(info_.joints[i].name,hardware_interface::HW_IF_VELOCITY,&v_[i]);
    }
    return out;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> out;
    for(size_t i=0;i<6;++i)
      out.emplace_back(info_.joints[i].name,hardware_interface::HW_IF_POSITION,&cmd_[i]);
    return out;
  }
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    try {bus_.open_port(port_);poll_all();active_=false;return hardware_interface::CallbackReturn::SUCCESS;}
    catch(const std::exception& e){
      RCLCPP_ERROR(rclcpp::get_logger("arm_hardware"),"configure: %s",e.what());
      bus_.close_port();return hardware_interface::CallbackReturn::ERROR;
    }
  }
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    try {poll_all();active_=true;return hardware_interface::CallbackReturn::SUCCESS;}
    catch(const std::exception& e){
      RCLCPP_ERROR(rclcpp::get_logger("arm_hardware"),"activate: %s",e.what());
      active_=false;return hardware_interface::CallbackReturn::ERROR;
    }
  }
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    active_=false;return hardware_interface::CallbackReturn::SUCCESS; // No torque release.
  }
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    active_=false;bus_.close_port();return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::return_type read(const rclcpp::Time&,const rclcpp::Duration&) override {
    try {
      size_t i=next_;next_=(next_+1)%6;poll(i);
      for(const auto& t:timestamps_)if(t.time_since_epoch().count()==0||
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-t).count()>max_age_)
        throw std::runtime_error("Servo position feedback is stale");
      return hardware_interface::return_type::OK;
    }catch(const std::exception& e){
      RCLCPP_ERROR(rclcpp::get_logger("arm_hardware"),"read: %s",e.what());
      return hardware_interface::return_type::ERROR;
    }
  }
  hardware_interface::return_type write(const rclcpp::Time&,const rclcpp::Duration&) override {
    if(!active_)return hardware_interface::return_type::OK;
    try {
      std::array<int,6> target{};
      bool changed=false;
      for(size_t i=0;i<6;++i){
        if(std::chrono::duration_cast<std::chrono::milliseconds>(
          Clock::now()-timestamps_[i]).count()>max_age_)
          throw std::runtime_error("Write blocked on stale state");
        target[i]=to_raw(i,cmd_[i]);
        int delta=std::abs(target[i]-last_sent_[i]);
        if(delta>step_)throw std::runtime_error(std::string(cal[i].name)+" command step too large");
        if(std::abs(target[i]-feedback_[i])>tracking_)
          throw std::runtime_error(std::string(cal[i].name)+" tracking mismatch");
        changed|=delta>=2;
      }
      if(!changed)return hardware_interface::return_type::OK;
      if(!motion_||!checked_)throw std::runtime_error("Motion disabled");
      for(size_t i=0;i<6;++i)if(std::abs(target[i]-last_sent_[i])>=2){
        bus_.move(int(i),target[i],duration_);
        last_sent_[i]=target[i];
      }
      return hardware_interface::return_type::OK;
    }catch(const std::exception& e){
      RCLCPP_ERROR(rclcpp::get_logger("arm_hardware"),"write: %s",e.what());
      return hardware_interface::return_type::ERROR;
    }
  }
private:
  double to_q(size_t i,int r)const{
    if(r<cal[i].low||r>cal[i].high)throw std::runtime_error("Feedback outside historical raw range");
    return i==5?finger_from_raw(r):cal[i].sign*(r-cal[i].zero)*rad_per_raw;
  }
  int to_raw(size_t i,double q)const{
    if(!std::isfinite(q))throw std::runtime_error("Nonfinite target");
    int r=i==5?finger_to_raw(q):int(std::lround(cal[i].zero+cal[i].sign*q/rad_per_raw));
    if(r<cal[i].low||r>cal[i].high)throw std::runtime_error("Target outside historical raw range");
    return r;
  }
  void poll(size_t i){
    auto now=Clock::now();
    int r=bus_.position(int(i),timeout_);
    double new_q=to_q(i,r);
    double dt=std::chrono::duration<double>(now-timestamps_[i]).count();
    v_[i]=timestamps_[i].time_since_epoch().count()!=0&&dt>0?(new_q-q_[i])/dt:0.0;
    q_[i]=new_q;feedback_[i]=r;timestamps_[i]=Clock::now();
    if(i==5){q_[6]=-q_[5];v_[6]=-v_[5];}
  }
  void poll_all(){
    for(size_t i=0;i<6;++i)poll(i);
    for(size_t i=0;i<6;++i){cmd_[i]=q_[i];last_sent_[i]=feedback_[i];}
    next_=0; // activation never commands movement
  }
  Serial bus_;
  std::string port_;
  std::array<double,7> q_{},v_{};
  std::array<double,6> cmd_{};
  std::array<int,6> feedback_{},last_sent_{};
  std::array<Clock::time_point,6> timestamps_{};
  size_t next_{0};
  int timeout_{40},max_age_{1200},step_{8},tracking_{80},duration_{180};
  bool motion_{false},checked_{false},active_{false};
};
}  // namespace arm_hardware
PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmSystem, hardware_interface::SystemInterface)
