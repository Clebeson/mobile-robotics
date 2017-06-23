#include <Aria/Aria.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <grpc++/grpc++.h>
#include "robot.grpc.pb.h"
#include "robot.pb.h"

using namespace std::chrono_literals;

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerWriter;
using grpc::Status;
using robot::Pose;
using robot::Velocity;
using robot::Telemetry;
using robot::SubscribeRequest;
using robot::Gateway;

class RobotGatewayImpl final : public Gateway::Service {
  ArRobot robot;
  ArRobotConnector connector;

  Status Subscribe(ServerContext* context, const SubscribeRequest* request,
                   ServerWriter<Telemetry>* writer) override {

    auto deadline = std::chrono::high_resolution_clock::now();
    while (!context->IsCancelled()) {
      Telemetry telemetry;
      auto pose = telemetry.mutable_pose();
      auto velocity = telemetry.mutable_velocity();
      
      robot.lock();
      pose->set_x(robot.getX());
      pose->set_y(robot.getY());
      pose->set_heading(robot.getTh() * std::asin(1) / 90.0);
      velocity->set_linear(robot.getVel());
      velocity->set_angular(robot.getRotVel() * std::asin(1) / 90.0);
      robot.unlock();
      
      writer->Write(telemetry);

      deadline += 100ms;
      std::this_thread::sleep_until(deadline);
    }

    return Status::OK;
  }

  Status Configure(ServerContext* context, const Telemetry* request, Telemetry* response) override {
    if (request->has_pose()) {
      ArPose pose;
      pose.setX(request->pose().x());
      pose.setY(request->pose().y());
      pose.setTh(request->pose().heading() * 90.0 / std::asin(1));
      
      robot.lock();
      robot.moveTo(pose);
      robot.unlock();
    }

    if (request->has_velocity()) {
      robot.lock();
      robot.setVel(request->velocity().linear());
      robot.setRotVel(request->velocity().angular() * 90.0 / std::asin(1));
      robot.unlock();
    }
    return Status::OK;
  }

 public:
  RobotGatewayImpl(ArArgumentParser* parser) : connector(parser, &robot) {
    if (!connector.connectRobot(&robot)) {
      Aria::logOptions();
      Aria::exit(1);
    }

    robot.enableMotors();
    robot.runAsync(true);
  }

  virtual ~RobotGatewayImpl() {
    // robot.stopRunning();
    // robot.waitForRunExit();
  }
};

void RunServer(ArArgumentParser* parser) {
  std::string server_address("0.0.0.0:50051");
  RobotGatewayImpl service(parser);
  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;
  server->Wait();
}

int main(int argc, char** argv) {
  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  // if (!Aria::parseArgs() || parser.checkHelpAndWarnUnparsed()) {
  //   ArLog::log(ArLog::Terse, "HODORRRRRRRRRRRRRRRRRRRR");
  //   Aria::logOptions();
  //   Aria::exit(1);
  // }

  RunServer(&parser);
  return 0;
}
