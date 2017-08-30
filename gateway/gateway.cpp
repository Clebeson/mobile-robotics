#include <Aria/Aria.h>
#include <spdlog/spdlog.h>
#include <boost/program_options.hpp>

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
using namespace boost::program_options;

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

class AriaDriver {
  ArRobot robot;

 public:
  AriaDriver(std::string const& serial) {
    auto* c = new ArSerialConnection();
    c->open(serial.c_str());
    connect(c);
  }

  AriaDriver(std::string const& hostname, int port) {
    auto* c = new ArTcpConnection();
    c->open(hostname.c_str(), port);
    connect(c);
  }

  template <typename Connection>
  void connect(Connection* c) {
    Aria::init();
    robot.setDeviceConnection(c);
    if (!robot.blockingConnect()) {
      throw std::runtime_error("Could not connect to robot.");
    }
    robot.runAsync(true);
  }

  virtual ~AriaDriver() {
    robot.stopRunning();
    robot.waitForRunExit();
  }

  void stop() {
    spdlog::get("robot")->info("Disabling motors");
    robot.lock();
    robot.disableMotors();
    robot.unlock();
  }

  void start() {
    spdlog::get("robot")->info("Enabling motors");
    robot.lock();
    robot.enableMotors();
    robot.unlock();
  }

  void set_pose(Pose const& pose) {
    ArPose ar_pose(pose.x(), pose.y(), pose.heading() * 90.0 / std::asin(1));
    robot.lock();
    robot.moveTo(ar_pose);
    robot.unlock();
  }

  void set_velocity(Velocity const& velocity) {
    robot.lock();
    robot.setVel(velocity.linear());
    robot.setRotVel(velocity.angular() * 90.0 / std::asin(1));
    robot.unlock();
  }

  Telemetry get_telemetry() {
    Telemetry telemetry;
    auto* pose = telemetry.mutable_pose();
    auto* velocity = telemetry.mutable_velocity();

    robot.lock();
    pose->set_x(robot.getX());
    pose->set_y(robot.getY());
    pose->set_heading(robot.getTh() * std::asin(1) / 90.0);
    velocity->set_linear(robot.getVel());
    velocity->set_angular(robot.getRotVel() * std::asin(1) / 90.0);
    robot.unlock();
    return telemetry;
  }
};

template <typename ThreadSafeDriver>
class RobotGatewayImpl final : public Gateway::Service {
  std::unique_ptr<ThreadSafeDriver> driver;
  std::mutex mutex;
  int n_consumers = 0;
  

  Status Subscribe(ServerContext* context, const SubscribeRequest* request,
                   ServerWriter<Telemetry>* writer) override {
    spdlog::get("robot")->info("New subscribe request");
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (n_consumers++ == 0)
        driver->start();
    }

    auto deadline = std::chrono::high_resolution_clock::now();
    while (!context->IsCancelled()) {
      writer->Write(driver->get_telemetry());
      deadline += 100ms;
      std::this_thread::sleep_until(deadline);
    }

    {
      std::lock_guard<std::mutex> lock(mutex);
      if (--n_consumers == 0)
        driver->stop();
    }

    return Status::OK;
  }

  Status Configure(ServerContext* context, const Telemetry* request, Telemetry* response) override {
    if (request->has_pose())
      driver->set_pose(request->pose());
    if (request->has_velocity())
      driver->set_velocity(request->velocity());
    return Status::OK;
  }

 public:
  template <typename... Args>
  RobotGatewayImpl(Args&&... args) : driver(std::make_unique<AriaDriver>(args...)) {}
  RobotGatewayImpl(std::unique_ptr<ThreadSafeDriver> driver) : driver(std::move(driver)) {}
};

variables_map parse_args(int argc, char** argv) {
  options_description aria{"Aria"};
  aria.add_options()
    ("tcp-host,t",    value<std::string>()->default_value("localhost"), "")
    ("tcp-port,p",    value<int>()->default_value(8101), "")
    ("serial-port,s", value<std::string>(), "");

  options_description grpc{"gRPC"};
  grpc.add_options()("address,a", value<std::string>()->default_value("0.0.0.0:50051"),
                     "gRPC server address");

  options_description all("Allowed options");
  all.add_options()("help,h", "produce this help message");
  all.add(aria).add(grpc);

  variables_map vm;
  store(parse_command_line(argc, argv, all), vm);

  if (vm.count("help")) {
    std::cout << all;
    exit(0);
  }
  return vm;
}

void run(RobotGatewayImpl<AriaDriver>* service, std::string const& address) {
  auto log = spdlog::stdout_color_mt("robot");

  ServerBuilder builder;
  builder.AddListeningPort(address, grpc::InsecureServerCredentials());
  builder.RegisterService(service);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  log->info(" server listening on {}", address);
  server->Wait();
};

int main(int argc, char** argv) {
  auto args = parse_args(argc, argv);

  auto address = args["address"].as<std::string>();
  if (args.count("serial-port")) {
    RobotGatewayImpl<AriaDriver> service(args["serial-port"].as<std::string>());
    run(&service, address);
  } else {
    RobotGatewayImpl<AriaDriver> service(args["tcp-host"].as<std::string>(),
                                         args["tcp-port"].as<int>());
    run(&service, address);
  }

  return 0;
}
