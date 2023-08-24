#include <dlib/gui_widgets.h>
#include <dlib/control.h>
#include <dlib/image_transforms.h>
#include <fstream>
#include <iostream>

using namespace std;
using namespace dlib;

//  ----------------------------------------------------------------------------
//ofstream outFile;
//ofstream fs;
//std::string filename = "output.csv";

int main()
{
    const int STATES = 4;
    const int CONTROLS = 2;

    matrix<double, STATES, STATES> A;
    A = 1, 0, 1, 0, // next_pos = pos + velocity
        0, 1, 0, 1, // next_pos = pos + velocity
        0, 0, 1, 0, // next_velocity = velocity
        0, 0, 0, 1; // next_velocity = velocity

    matrix<double, STATES, CONTROLS> B;
    B = 0, 0,
    0, 0,
    1, 0,
    0, 1;

    matrix<double, STATES, 1> C;
    C = 0,
    0,
    0,
    0.1;

    const int HORIZON = 30;

    matrix<double, STATES, 1> Q;
    // Setup Q so that the MPC only cares about matching the target position and
    // ignores the velocity.
    Q = 1, 1, 0, 0;

    matrix<double, CONTROLS, 1> R, lower, upper;
    R = 1, 1;
    lower = -0.5, -0.5;
    upper = 0.5, 0.5;

    mpc<STATES, CONTROLS, HORIZON> controller(A, B, C, Q, R, lower, upper);

    // Let's tell the controller to send our vehicle to a random location.  It
    // will try to find the controls that makes the vehicle just hover at this
    // target position.
    dlib::rand rnd;
    matrix<double, STATES, 1> target;
    target = rnd.get_random_double() * 400, rnd.get_random_double() * 400, 0, 0;
    controller.set_target(target);

    matrix<double, STATES, 1> current_state;
    // And we start it at the center of the world with zero velocity.
    current_state = 200, 200, 0, 0;
    //fs.open(outFile, filename);
    std::ofstream outFile("output.csv");
    int iter = 0;
    while (iter < 100)
    {
        // Find the best control action given our current state.
        matrix<double, CONTROLS, 1> action = controller(current_state);
        cout << "best control: " << trans(action);
        current_state = A * current_state + B * action + C;
        outFile << current_state << "\n";
        dlib::sleep(100);
        iter++;
    }
    outFile.close();
    return 0;
}

//  ----------------------------------------------------------------------------
