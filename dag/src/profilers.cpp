#include <ros/ros.h>
#include <sys/types.h>
#include <sys/sysinfo.h>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <profilers.h>
#include <my_demo/ProfilersInfo.h>
/**
 * @brief The MemProfiler class
 * @author jixiang
 */

Profilers::MemProfiler::MemProfiler()
{
  struct sysinfo memInfo;
  sysinfo (&memInfo);

  totalPhysMem = memInfo.totalram * memInfo.mem_unit;
  physMemFree = memInfo.freeram * memInfo.mem_unit;
  physMemUsed = totalPhysMem - physMemFree;
  totalVirtualMem = memInfo.totalswap * memInfo.mem_unit + totalPhysMem;
  virtualMemFree = memInfo.freeswap * memInfo.mem_unit + physMemFree;
  virtualMemUsed = totalVirtualMem - virtualMemFree;
}

long long Profilers::MemProfiler::getTotalPhysMem() { return totalPhysMem; }
long long Profilers::MemProfiler::getPhysMemFree() { return physMemFree; }
long long Profilers::MemProfiler::getPhysMemUsed() { return physMemUsed; }
long long Profilers::MemProfiler::getTotalVirtualMem() { return totalVirtualMem;}
long long Profilers::MemProfiler::getVirtualMemFree() { return virtualMemFree; }
long long Profilers::MemProfiler::getVirtualMemUsed() { return virtualMemUsed; }

std::vector<size_t> Profilers::CPUProfiler::readProcStat()
{
  std::ifstream proc_stat ("/proc/stat");
  std::vector<size_t> states;
  proc_stat.ignore(5, ' '); //ignor the "cpu  "

  for (size_t time; proc_stat >> time; states.push_back(time));
  proc_stat.close();
  ROS_INFO_STREAM ("---Total CPU Time---" <<std::endl <<
                   "User | Nice | System | Idle");
  ROS_INFO_STREAM (states[0] << " | " << states[1] << " | " << states[2]
                             << " | " << states[3] << " | ");
  return states;
}

double Profilers::CPUProfiler::getPercentUsed(int period)
{
  std::vector<size_t> last, now;
  double percent;

  // read the "proc/stat"
  last= readProcStat();

  // sleep for a cetain period then read
  sleep(period);
  now = readProcStat();

  if (now[0] < last[0] || now[1] < last[1] ||
      now[2] < last[2] || now[3] < last[3])
  {
    // overflow detection. Just skip this value.
    percent = -1;
  } else
  {
    double total = (now[0] - last[0]) + (now[1] - last[1]) +
        (now[2] - last[2]);
    percent = total;
    total += (now[3] - last[3]);
    percent /= total;
    percent *= 100.0;
  }
  return percent;
}

int Profilers::ProgramProfiler::getVirtualMemConsumed()
{
  FILE* file = fopen("/proc/self/status", "r");
  int result = -1;
  char line[128];

  while (fgets(line, 128, file) != NULL){
    if (strncmp(line, "Name:", 5) == 0){
      ROS_INFO_STREAM(line);
    }
    if (strncmp(line, "VmSize:", 7) == 0){
      result = parseLine(line);
      break;
    }
  }
  fclose(file);
  virtualMemConsumed = result;
  return virtualMemConsumed;
}

int Profilers::ProgramProfiler::getPhysMemConsumed()
{
  FILE* file = fopen("/proc/self/status", "r");
  int result = -1;
  char line[128];

  while (fgets(line, 128, file) != NULL){
    if (strncmp(line, "VmRSS:", 6) == 0){
      result = parseLine(line);
      break;
    }
  }
  fclose(file);
  virtualMemConsumed = result;
  return virtualMemConsumed;
}

int Profilers::ProgramProfiler::parseLine(char *line)
{
  // This assumes that a digit will be found and the line ends in " Kb".
  int i = strlen(line);
  const char* p = line;
  while (*p <'0' || *p > '9') p++;
  line[i-3] = '\0';
  i = atoi(p);
  return i;
}

int main(int argc, char **argv)
{


  // Set up ROS.
  ros::init(argc, argv, "profilers");

  Profilers profilers;
  double seconds = 1.0;


  ROS_INFO_STREAM("*** Memory Usage Info *** ");
  ROS_INFO_STREAM("Total Physical Memory: " << profilers.memProfiler.getTotalPhysMem());
  ROS_INFO_STREAM("Total Virutal Memory: " << profilers.memProfiler.getTotalVirtualMem());
  ROS_INFO_STREAM("Free Physical Memory: " << profilers.memProfiler.getPhysMemFree());
  ROS_INFO_STREAM("Free Virtual Memory: " << profilers.memProfiler.getVirtualMemFree());
  ROS_INFO_STREAM("Currently Used Physical Memory: " << profilers.memProfiler.getPhysMemUsed());
  ROS_INFO_STREAM("Currently Used Virtual Memory: " << profilers.memProfiler.getVirtualMemUsed());

  ROS_INFO_STREAM("*** CPU Usage Info ***");
  ROS_INFO_STREAM("Total CPU Usage Percentage: " << profilers.cpuProfiler.getPercentUsed(seconds) <<"%");

  ROS_INFO_STREAM("***Program Profiler***");
  ROS_INFO_STREAM("Virtual Memory Consumed by current process: "
                  << profilers.programProfiler.getVirtualMemConsumed() << " KB");
  ROS_INFO_STREAM("Physical Memory Consumed by current process: "
                  << profilers.programProfiler.getVirtualMemConsumed() << " KB");

  ros::NodeHandle n;
  ros::Publisher profileReporter = n.advertise<my_demo::ProfilersInfo>
      ("profile_reporter", 1000);

  ros::Rate loop_rate(10);
  /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
  int count = 0;
  while (ros::ok())
  {
    /**
       * This is a message object. You stuff it with data, and then publish it.
       */

    my_demo::ProfilersInfo msg;

    msg.totalVirtualMem = profilers.memProfiler.getTotalVirtualMem();
    msg.totalPhysMem = profilers.memProfiler.getTotalPhysMem();
    msg.virtualMemFree = profilers.memProfiler.getVirtualMemFree();
    msg.physMemFree = profilers.memProfiler.getPhysMemFree();
    msg.virtualMemUsed = profilers.memProfiler.getVirtualMemUsed();
    msg.physMemUsed = profilers.memProfiler.getPhysMemUsed();
    msg.cpuUsage = profilers.cpuProfiler.getPercentUsed(seconds);

    // Publich the profiled information
    profileReporter.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
