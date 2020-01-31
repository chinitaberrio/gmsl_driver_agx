#include "PrintEventHandler.h"
#include <iostream>

PrintEventHandler::PrintEventHandler()
  : is_running_(false) {
  future_ = std::async(std::launch::async, [&]() {
    base_.loopForever();
  });
}

void PrintEventHandler::Print(const std::string &text) {
  base_.runInEventBaseThread([&]() {
    std::cout << text << std::endl;
  });
}

void PrintEventHandler::Print(const std::string &author, const std::string &text) {
  base_.runInEventBaseThread([&]() {
    std::cout << author << " said: " << text << std::endl;
  });
}


void PrintEventHandler::Stop() {
  std::cout << "PrintEventHandler Stop is called." << std::endl;
  base_.terminateLoopSoon();
  future_.get();
  std::cout << "PrintEventHandler is dead." << std::endl;
}

PrintEventHandler::~PrintEventHandler() {
  base_.terminateLoopSoon();
}
