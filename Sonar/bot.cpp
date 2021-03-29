//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include "bot.h"
#include <glog/logging.h>

namespace PIRoBot {
  Bot::Bot(): pBotThread(nullptr), isExitThread(false), isNewState(false) {};
  void Bot::StartBotThread() {
    LOG(INFO)<<"start bot thread";
    pBotThread = new std::thread(&Bot::Run, this);
  }
  void Bot::EndBotThread() {
    LOG(INFO)<<"end bot thread";
    pBotThread->join();
    delete pBotThread;
    pBotThread = nullptr;
  }
  void Bot::Run() {
    while(1) {
      mutexExitThread.lock();
      if(isExitThread) {
        mutexExitThread.unlock();
        break;
      }
      mutexExitThread.unlock();

      this->RunOnce();
      usleep(1000);
    }
    LOG(INFO)<<"exit Bot thread";
  }
}
