// Exercise the real serial parser against a pseudo-terminal, never real hardware.
#include "../src/arm_system.cpp"
#include <pty.h>
#include <thread>
#include <iostream>
int main() {
  int master, slave; char name[128];
  if (openpty(&master, &slave, name, nullptr, nullptr)) return 2;
  arm_hardware::Serial serial; serial.open_port(name); close(slave);
  std::thread servo([&] {
    std::string input; int queries=0;
    for (;;) {
      pollfd p{master,POLLIN,0};
      if (::poll(&p,1,500)<=0) return;
      char c; if (::read(master,&c,1)!=1) return;
      input+=c;
      if(c=='!') {
        if(input=="#000PRAD!") {
          ++queries;
          if(queries==2) { const char reply[]="#000P1500!";
            ::write(master,reply,sizeof(reply)-1); return; }
        }
        input.clear();
      }
    }
  });
  bool recovered=false;
  try { recovered=serial.position(0,40)==1500; } catch(const std::exception& e) { std::cerr<<e.what()<<"\n"; }
  servo.join();
  if(!recovered) { close(master); std::cerr<<"FAIL: one lost response must recover\n"; return 1; }
  auto start=std::chrono::steady_clock::now(); bool exhausted=false;
  try { serial.position(0,40); } catch(const std::exception&) { exhausted=true; }
  auto elapsed=std::chrono::steady_clock::now()-start;
  if(!exhausted || elapsed>std::chrono::milliseconds(300)) return 3;
  close(master); bool disconnected=false;
  try { serial.position(0,40); } catch(const std::exception&) { disconnected=true; }
  if(!disconnected) return 4;
  std::cout<<"PASS: isolated timeout recovered; persistent loss and disconnect fail\n";
}
