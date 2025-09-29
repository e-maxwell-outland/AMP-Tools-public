// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpace2DConstructor.h"
#include "MyManipulator2D.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // -------- Forward Kinematics --------
    // Create manipulator based on link lengths
    std::vector<double> link_lengths {0.5, 1.0, 0.5};
    MyManipulator2D fk_manipulator(link_lengths);

    // Define angle states
    auto pi = M_PI;
    amp::ManipulatorState test_state_1 = Eigen::Vector3d(pi/6, pi/3, (pi*7)/4);

    // Forward kinematics + visualize solution (Exercise 2a)
    Visualizer::makeFigure(fk_manipulator, test_state_1);

    // -------- Inverse Kinematics --------
    // Create manipulator based on link lengths
    std::vector<double> ik_link_lengths {1.0, 0.5, 1.0};
    MyManipulator2D ik_manipulator(ik_link_lengths);

    // Find angles to reach end effector state (Exercise 2b)
    Eigen::Vector2d end_effector(2,0);
    amp::ManipulatorState test_state_2 = ik_manipulator.getConfigurationFromIK(end_effector);

    // Forward kinematics + visualize solution
    Visualizer::makeFigure(ik_manipulator, test_state_2);

    // Create the collision space constructor
    std::size_t n_cells = 120;
    MyManipulatorCSConstructor cspace_constructor(n_cells);
    MyManipulator2D manipulator;

    // C-Space for Exercise 3a
    std::unique_ptr<amp::GridCSpace2D> cspace1 = cspace_constructor.construct(manipulator, HW4::getEx3Workspace1());
    Visualizer::makeFigure(*cspace1);
    Visualizer::makeFigure(HW4::getEx3Workspace1());

    // C-Space for Exercise 3b
    std::unique_ptr<amp::GridCSpace2D> cspace2 = cspace_constructor.construct(manipulator, HW4::getEx3Workspace2());
    Visualizer::makeFigure(*cspace2);
    Visualizer::makeFigure(HW4::getEx3Workspace2());

    // C-Space for Exercise 3c
    std::unique_ptr<amp::GridCSpace2D> cspace3 = cspace_constructor.construct(manipulator, HW4::getEx3Workspace3());
    Visualizer::makeFigure(*cspace3);
    Visualizer::makeFigure(HW4::getEx3Workspace3());

    // Save and plot all created figures
    Visualizer::saveFigures();

    // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "emily.maxwell@colorado.edu", argc, argv);
    return 0;
}