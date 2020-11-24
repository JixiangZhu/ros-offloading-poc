#ifndef PROFILERS_H
#define PROFILERS_H

class Profilers{
private:
  class MemProfiler
  {
  private:
    long long totalPhysMem;
    long long physMemFree;
    long long physMemUsed;
    long long totalVirtualMem;
    long long virtualMemFree;
    long long virtualMemUsed;

  public:
    MemProfiler();

    long long getTotalPhysMem();
    long long getPhysMemFree();
    long long getPhysMemUsed();
    long long getTotalVirtualMem();
    long long getVirtualMemFree();
    long long getVirtualMemUsed();
  };

  class CPUProfiler
  {
  private:
    double percentUsed;  //current used cpu percentage
    std::vector<size_t> readProcStat();

  public:
    double getPercentUsed(int period);
  };

  class ProgramProfiler
  {

  private:
    int virtualMemConsumed;
    int physMemConsumed;
    double cpuConsumed;

  public:
    int getVirtualMemConsumed();
    int getPhysMemConsumed();
    double getCpuConsumed();

  private:
    int parseLine(char* line);
  };
  
  class NetworkProfiler
  {
  private:
      std::vector<double> speeds;
  public:
      std::vector<double> getSpeeds();
  };

public:
  MemProfiler memProfiler;
  CPUProfiler cpuProfiler;
  ProgramProfiler programProfiler;
};




#endif // PROFILERS_H
