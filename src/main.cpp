#include <array>
#include <iostream>
#include <math.h>
#include <string>

#include <uWS/uWS.h>
#include "json.hpp"

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

static constexpr double TWIDDLE_TOLERANCE = 0.1;
static constexpr unsigned TWIDDLE_INCREASE_IGNORE_PERIOD_EVERY_N_CYCLES = 18u;  // 6u == 2 runs per parameter, that is the whole twiddle algorithm.
static constexpr unsigned TWIDDLE_IGNORE_FIRST_N_TICKS = 25u;
static constexpr unsigned TWIDDLE_AVERAGE_ERROR_OVER_N_TICKS = 100u;

int main()
{
    uWS::Hub h;

    PID pid;
    Twiddle twiddle(TWIDDLE_TOLERANCE);

    // Best found params go here:
    // pid.UpdateParams({1, 0, 1.05279});

    // Restart twiddle here:
    // twiddle.SetCoefficients({0.0395954, 0.0237573, 0.0527939});

    bool enableTwiddle = true;

    std::vector<double> pidParams = pid.GetParams();

    std::array<double, TWIDDLE_AVERAGE_ERROR_OVER_N_TICKS> errors;
    std::array<double, TWIDDLE_AVERAGE_ERROR_OVER_N_TICKS> speeds;
    unsigned twiddleIgnoreFirstNTicks { TWIDDLE_IGNORE_FIRST_N_TICKS };
    unsigned int twiddleTick { 0u };

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
            if (enableTwiddle && twiddleTick == twiddleIgnoreFirstNTicks + TWIDDLE_AVERAGE_ERROR_OVER_N_TICKS)
            {
                static unsigned twiddleCycle { 0 };
                double averageEndError = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
                double averageEndSpeed = std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size() + 0.01;
                double finalError = averageEndError / averageEndSpeed;

                bool twiddleDone = twiddle.runOnce(finalError, pidParams);
                pid.UpdateParams(pidParams);  // Could be referenced in instead

                std::cout << "PID params: " << pidParams[0] << ", " << pidParams[1] << ", " << pidParams[2] << std::endl;
                const auto twiddleCoeffs = twiddle.GetCoefficients();
                std::cout << "Twiddle coefficients: " << twiddleCoeffs[0] << ", " << twiddleCoeffs[1] << ", " << twiddleCoeffs[2] << std::endl;

                string msg = "42[\"reset\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                twiddleTick = 0u;

                if (twiddleDone)
                {
                    enableTwiddle = false;
                    std::cout << "Twiddle complete. Final params: " << pidParams[0] << ", " << pidParams[1] << ", " << pidParams[2] << std::endl;
                }

                twiddleCycle++;
                if (twiddleCycle % TWIDDLE_INCREASE_IGNORE_PERIOD_EVERY_N_CYCLES)
                {
                    twiddleIgnoreFirstNTicks += TWIDDLE_IGNORE_FIRST_N_TICKS;
                }
            }

            // j[1] is the data JSON object
            double cte = std::stod(j[1]["cte"].get<string>());
            double speed = std::stod(j[1]["speed"].get<string>());
            // double angle = std::stod(j[1]["steering_angle"].get<string>());

            if (enableTwiddle && twiddleTick >= twiddleIgnoreFirstNTicks)
            {
                errors[twiddleTick - twiddleIgnoreFirstNTicks] = cte;
                speeds[twiddleTick - twiddleIgnoreFirstNTicks] = speed;
            }

            double steer_value = pid.Apply(cte);

            // DEBUG
            // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            twiddleTick++;
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}
