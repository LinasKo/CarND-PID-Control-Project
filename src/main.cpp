#include <array>
#include <iostream>
#include <limits>
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
static constexpr unsigned TWIDDLE_INCREASE_IGNORE_PERIOD_EVERY_N_CYCLES = 6u;  // 6u == 2 runs per parameter, that is the whole twiddle algorithm.
static constexpr unsigned TWIDDLE_IGNORE_FIRST_N_TICKS = 200u;
static constexpr unsigned TWIDDLE_AVERAGE_ERROR_OVER_N_TICKS = 400u;

static constexpr unsigned ALLOW_ALL_IN_FIRST_N_TICKS = 100u;
static constexpr double MAX_ALLOWED_CTE = 4.0;
static constexpr double MIN_ALLOWED_SPEED = 5.0;

int main()
{
    uWS::Hub h;

    PID pid;
    // Best found params go here:
    // pid.UpdateParams({1, 0, 1.05279});

    bool enableTwiddle = true;

    Twiddle twiddle(TWIDDLE_TOLERANCE);
    // Set initial twiddle coefficients:
    twiddle.SetCoefficients({0.1, 0.1, 0.1});


    std::vector<double> pidParams = pid.GetParams();

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
                static unsigned twiddleTick { 0u };  // Computes how many times telemetry event was received, resets when twiddle is run
                static unsigned twiddleIgnoreFirstNTicks { TWIDDLE_IGNORE_FIRST_N_TICKS };

                static std::array<double, TWIDDLE_AVERAGE_ERROR_OVER_N_TICKS> errors;
                static std::array<double, TWIDDLE_AVERAGE_ERROR_OVER_N_TICKS> speeds;

                bool runTwiddleNow = false;
                double finalError;

                if (twiddleTick >= ALLOW_ALL_IN_FIRST_N_TICKS && (cte > MAX_ALLOWED_CTE || speed < MIN_ALLOWED_SPEED))
                {
                    // Terminate early
                    std::cout << "Terminating early." << std::endl;
                    finalError = std::numeric_limits<double>::max();
                    runTwiddleNow = true;
                }
                else if (twiddleTick == twiddleIgnoreFirstNTicks + TWIDDLE_AVERAGE_ERROR_OVER_N_TICKS)
                {
                    // Run twiddle normally
                    const double averageEndError = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
                    const double averageEndSpeed = std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
                    finalError = averageEndError / (averageEndSpeed + 0.01);
                    runTwiddleNow = true;
                }

                if (runTwiddleNow)
                {
                    static unsigned twiddleCycle { 0u };  // How many times twiddle was run

                    const auto prevParams = pid.GetParams();
                    const bool twiddleDone = twiddle.runOnce(finalError, pidParams);
                    pid.UpdateParams(pidParams);  // Could be referenced in instead
                    if (prevParams == pidParams)
                    {
                        std::cout << ">>>>> Found better PID params: " << pidParams[0] << ", " << pidParams[1] << ", " << pidParams[2] << std::endl;
                    }
                    else
                    {
                        std::cout << "Trying PID params: " << pidParams[0] << ", " << pidParams[1] << ", " << pidParams[2] << std::endl;
                    }

                    const auto twiddleCoeffs = twiddle.GetCoefficients();
                    std::cout << "Twiddle coefficients: " << twiddleCoeffs[0] << ", " << twiddleCoeffs[1] << ", " << twiddleCoeffs[2] << std::endl;

                    // Reset simulator
                    const string msg = "42[\"reset\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);


                    if (twiddleDone)
                    {
                        enableTwiddle = false;
                        std::cout << "Twiddle complete. Final params: " << pidParams[0] << ", " << pidParams[1] << ", " << pidParams[2] << std::endl;
                    }

                    // Increase distance that the car needs to travel before twiddle starts looking at the errors
                    twiddleCycle++;
                    if (twiddleCycle % TWIDDLE_INCREASE_IGNORE_PERIOD_EVERY_N_CYCLES)
                    {
                        twiddleIgnoreFirstNTicks += TWIDDLE_IGNORE_FIRST_N_TICKS;
                    }

                    twiddleTick = 0u;
                }

                if (twiddleTick >= twiddleIgnoreFirstNTicks)
                {
                    errors[twiddleTick - twiddleIgnoreFirstNTicks] = cte;
                    speeds[twiddleTick - twiddleIgnoreFirstNTicks] = speed;
                }

                twiddleTick++;
            }  // end if(enableTwiddle)

            double steer_value = pid.Apply(cte);

            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Speed: " << speed << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
