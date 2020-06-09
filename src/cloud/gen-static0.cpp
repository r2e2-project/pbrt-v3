#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <set>
#include <vector>

#include "geometry.h"
#include "accelerators/cloud.h"
#include <pbrt/main.h>
#include <dirent.h>

using namespace std;
using namespace pbrt;

void surfaceArea(const string &path, ofstream &static0) {
  pbrt::PbrtOptions.nThreads = 1;
  pbrt::scene::Base base(path, 1);

  size_t numTreelets = base.GetTreeletCount();
  static0 << numTreelets << endl;

  vector<uint64_t> treeletAreas;
  vector<uint64_t> unionAreas;
  vector<set<uint32_t>> treeletChildren;
  vector<uint64_t> instanceAreas(numTreelets);
  vector<uint64_t> finalAreas(numTreelets);

  for (uint32_t treeletIdx = 0; treeletIdx < numTreelets; treeletIdx++) {
      auto treelet = pbrt::scene::LoadTreelet(path, treeletIdx);
      uint64_t area = (uint64_t)treelet->RootSurfaceAreas();
      uint64_t unionArea = (uint64_t)treelet->SurfaceAreaUnion();
      treeletAreas.push_back(area);
      unionAreas.push_back(unionArea);

      auto info = treelet->GetInfo(treeletIdx);

      for (auto &kv : info.instances) {
          instanceAreas[kv.first] += kv.second;
      }

      treeletChildren.push_back(info.children);
  }

  for (uint32_t treeletIdx = 0; treeletIdx < numTreelets; treeletIdx++) {
      uint64_t instanceArea = instanceAreas[treeletIdx];
      uint64_t treeletArea = treeletAreas[treeletIdx];
      if (instanceArea == 0) {
          finalAreas[treeletIdx] = treeletArea;
          continue;
      }

      vector<uint32_t> areaStack {treeletIdx};
      while (!areaStack.empty()) {
          uint32_t subInst = areaStack.back();
          areaStack.pop_back();

          uint32_t subInstArea = treeletAreas[subInst];
          double areaRatio = subInstArea / treeletArea;

          finalAreas[subInst] = areaRatio * instanceArea;

          for (uint32_t child : treeletChildren[subInst]) {
              areaStack.push_back(child);
          }
      }
  }

  for (uint32_t treeletIdx = 0; treeletIdx < numTreelets; treeletIdx++) {
      uint64_t areaHeuristic = (uint64_t)ceil(finalAreas[treeletIdx]);

      cout << treeletIdx << " " << treeletAreas[treeletIdx] << " " <<
              unionAreas[treeletIdx] << " " <<
              areaHeuristic << endl;

      static0 << areaHeuristic << " " << 1 << " " << treeletIdx << endl;
  }
}

void randomRays(const string &path, ofstream &static0) {
    DIR *dir = opendir(path.c_str());
    if (!dir) {
        cerr << "Can't open " << path << endl;
        exit(EXIT_FAILURE);
    }

    dirent *entry = nullptr;
    while ((entry = readdir(dir))) {
        if (entry->d_name[0] != 'B') continue;
        printf("%s\n", entry->d_name);
    }

    closedir(dir);
}

int main(int argc, char* argv[])
{
  if (argc < 4) {
      cerr << "Usage: gen-static0 CMD SCENE STATIC0" << endl;
      return EXIT_FAILURE;
  }

  const string cmd { argv[1] };
  const string path { argv[2] };
  ofstream static0(argv[3]);

  if (cmd == "SAH") {
      surfaceArea(path, static0);
  } else {
      randomRays(path, static0);
  }

  return EXIT_SUCCESS;
}
