#include <array>
#include <iostream>
#include <limits>
#include <math.h>
#include <string>

#include <uWS/uWS.h>
#include "json.hpp"
#include "spdlog/spdlog.h"

#include "Twiddle.h"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;


using namespace pid_control;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

// Twiddle configuration
static constexpr double TWIDDLE_TOLERANCE = 0.1;

static constexpr unsigned ALLOW_ALL_IN_FIRST_N_TICKS = 100u;
static constexpr double MAX_ALLOWED_CTE = 4.0;
static constexpr double MIN_ALLOWED_SPEED = 5.0;
static constexpr unsigned TERMINATE_AFTER_N_TICKS = 1000u;

int main()
{
    uWS::Hub h;
    spdlog::set_level(spdlog::level::info);

    PID pid;
    // Best found params go here:
    // pid.UpdateParams({1, 0, 1.05279});
    std::vector<double> pidParams = pid.GetParams();

    bool enableTwiddle = true;

    Twiddle twiddle(TWIDDLE_TOLERANCE);
    // Set initial twiddle coefficients:
    twiddle.SetCoefficients({0.1, 0.1, 0.1});

    h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
    {
        if (not (length && length > 2 && data[0] == '4' && data[1] == '2'))
        {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message
            // The 2 signifies a websocket event
            return;
        }

        auto s = hasData(string(data).substr(0, length));
        if (s == "")
        {
            // Manual driving
            string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            return;
        }

        auto j = json::parse(s);
        string event = j[0].get<string>();

        if (event == "telemetry")
        {
            // j[1] is the data JSON object
            double cte = std::stod(j[1]["cte"].get<string>());
            double speed = std::stod(j[1]["speed"].get<string>());
            // double angle = std::stod(j[1]["steering_angle"].get<string>());

            if (enableTwiddle)
            {
                static unsigned twiddleTick { 0u };  // Computes how many ticks have passed since twiddle was called

                const bool ranVeryLong = twiddleTick >= TERMINATE_AFTER_N_TICKS;
                const bool errorTooLarge = cte > MAX_ALLOWED_CTE;
                const bool gotTooSlow = speed < MIN_ALLOWED_SPEED;
                if (twiddleTick >= ALLOW_ALL_IN_FIRST_N_TICKS && (ranVeryLong || errorTooLarge || gotTooSlow))
                {
                    // Convert run time to error that twiddle expects
                    const double twiddleError = std::numeric_limits<unsigned>::max() - twiddleTick;

                    const auto prevParams = pid.GetParams();
                    const bool twiddleDone = twiddle.runOnce(twiddleError, pidParams);
                    pid.UpdateParams(pidParams);  // Could be a reference instead, maybe
                    if (prevParams == pidParams)
                    {
                        spdlog::warn("Found better PID params: {}, {}, {}", pidParams[0], pidParams[1], pidParams[2]);
                    }
                    else
                    {
                        spdlog::info("Trying PID params: {}, {}, {}", pidParams[0], pidParams[1], pidParams[2]);
                    }

                    const auto twiddleCoeffs = twiddle.GetCoefficients();
                    spdlog::info("Twiddle coefficients: {}, {}, {}", twiddleCoeffs[0], twiddleCoeffs[1], twiddleCoeffs[2]);

                    // Reset simulator
                    const string msg = "42[\"reset\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    // Maybe terminate Twiddle
                    if (ranVeryLong)
                    {
                        enableTwiddle = false;
                        spdlog::warn("Managed to run long enough! Terminating Twiddle.");
                        spdlog::warn("Final params: {}, {}, {}", pidParams[0], pidParams[1], pidParams[2]);
                    }
                    if (twiddleDone)
                    {
                        enableTwiddle = false;
                        spdlog::warn("Twiddle tolerance reached! Terminating Twiddle.");
                        spdlog::warn("Final params: {}, {}, {}", pidParams[0], pidParams[1], pidParams[2]);
                    }

                    twiddleTick = 0u;
                }

                twiddleTick++;
            }  // end if(enableTwiddle)

            double steer_value = pid.Apply(cte);

            // DEBUG
            spdlog::debug("CTE: {}, Steering Value: {}, Speed: {}", cte, steer_value, speed);

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            spdlog::debug("Message: {}", msg);
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        spdlog::debug("Connected!!!");
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
        ws.close();
        spdlog::info("Disconnected");
    });

    int port = 4567;
    if (h.listen(port))
    {
        spdlog::info("Listening to port {}", port);
    }
    else
    {
        spdlog::error("Failed to listen to port");
        return -1;
    }

    h.run();
}
