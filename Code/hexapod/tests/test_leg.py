from hexapod import leg
import numpy as np

def test_legPos():
    """Tests that the output positions of the leg segments is correct given the coax, femur, and tibia angles with the body model."""
    body_model = np.array([[42.5, 73.61215932, 0],
                            [85, 0, 0],
                            [42.5, -73.61215932, 0],
                            [-42.5, -73.61215932, 0],
                            [-85, 0, 0],
                            [-42.5, 73.61215932, 0],
                            [42.5, 73.61215932, 0]])

    assert np.allclose(leg.legPos(60, 30, -90, body_model, 1), np.array([[85, 0, 0],
                        [98.17, 22.81110914, 0],
                        [131.16556788, 79.96110914, 38.1],
                        [92.92188605, 13.72110914, -6.06]]))


def test_legAngle():
    """Tests that the coax, femur, and tibia angles are correct given the end foot position. This test uses the defult leg dimensions declared by the function definition"""
    assert np.allclose(leg.legAngle(40, 10, -20), np.array([14.036243467926479, 57.837989407032495, -74.73564115953161]))

def test_legAngleZeros():
    """Tests that the coax, femur, and tibia angles are correct given the end foot position. This test uses the defult leg dimensions declared by the function definition"""
    assert np.allclose(leg.legAngle(26.34 + 76.2, 0, -88.32),
                       np.array([0, 0, 0]))

def test_recalculateLegAngles():
    """Tests that all of the output leg servo angles are correct given the default body model and feet positions when the hexapod is just turned on"""
    feet_positions = np.array([[75, 129.903811, -20],
                                [150, 0, -20],
                                [75, -129.903811, -20],
                                [-75, -129.903811, -20],
                                [-150, 0, -20],
                                [-75, 129.903811, -20]])

    body_model = np.array([[42.5, 73.61215932, 0],
                            [85, 0, 0],
                            [42.5, -73.61215932, 0],
                            [-42.5, -73.61215932, 0],
                            [-85, 0, 0],
                            [-42.5, 73.61215932, 0],
                            [42.5, 73.61215932, 0]])

    assert np.allclose(leg.recalculateLegAngles(feet_positions, body_model),
    np.array([[60, 63.50456104, -60.47672858],
            [0, 63.50456104, -60.47672858],
            [-60, 63.50456104, -60.47672858],
            [-120, 63.50456104, -60.47672858],
            [180, 63.50456104, -60.47672858],
            [120, 63.50456104, -60.47672858]]))

def test_startLegPos():
    """Tests that the starting leg angles are correct when given the body model of the starting condition of the robot"""
    body_model = np.array([[42.5, 73.61215932, 0],
                            [85, 0, 0],
                            [42.5, -73.61215932, 0],
                            [-42.5, -73.61215932, 0],
                            [-85, 0, 0],
                            [-42.5, 73.61215932, 0],
                            [42.5, 73.61215932, 0]])

    assert np.allclose(leg.startLegPos(body_model, 150, 20), np.array([[60, 63.50456104, -60.47672858],
                                                            [0, 63.50456104, -60.47672858],
                                                            [-60, 63.50456104, -60.47672858],
                                                            [-120, 63.50456104, -60.47672858],
                                                            [180, 63.50456104, -60.47672858],
                                                            [120, 63.50456104, -60.47672858]]))

def test_legModel():
    """Tests that the found model of the legs is correct given the leg angles and body model of the starting condition of the hexapod"""
    leg_angles = np.array([[60, 63.50456104, -60.47672858],
                            [0, 63.50456104, -60.47672858],
                            [-60, 63.50456104, -60.47672858],
                            [-120, 63.50456104, -60.47672858],
                            [180, 63.50456104, -60.47672858],
                            [120, 63.50456104, -60.47672858]])

    body_model = np.array([[42.5, 73.61215932, 0],
                            [85, 0, 0],
                            [42.5, -73.61215932, 0],
                            [-42.5, -73.61215932, 0],
                            [-85, 0, 0],
                            [-42.5, 73.61215932, 0],
                            [42.5, 73.61215932, 0]])

    assert np.allclose(leg.legModel(leg_angles, body_model),
    np.array([[[42.5, 85, 42.5, -42.5, -85, -42.5],
                [73.6121593, 0, -73.6121593, -73.6121593, 0, 73.6121593],
                [0, 0, 0, 0, 0, 0]],

                [[55.67, 111.34,  55.67, -55.67, -111.34, -55.67],
                [96.4232685, 0, -96.4232685, -96.4232685, 0, 96.4232685],
                [0, 0, 0, 0, 0, 0]],

                [[72.6674223, 145.334845, 72.6674223, -72.6674223, -145.334845, -72.6674223],
                [125.863668, 0, -125.863668, -125.863668, 0, 125.863668],
                [68.1967047, 68.1967047, 68.1967047, 68.1967047, 68.1967047, 68.1967047]],

                [[75, 150, 75, -75, -150, -75],
                [129.903811, 0, -129.903811, -129.903811, 0, 129.903811],
                [-20, -20, -20, -20, -20, -20]]]))

def test_getFeetPos():
    """Tests that the found feet positions as x, y, z points are correct given the model of the legs"""
    leg_model = np.array([[[42.5, 85, 42.5, -42.5, -85, -42.5],
                [73.6121593, 0, -73.6121593, -73.6121593, 0, 73.6121593],
                [0, 0, 0, 0, 0, 0]],

                [[55.67, 111.34,  55.67, -55.67, -111.34, -55.67],
                [96.4232685, 0, -96.4232685, -96.4232685, 0, 96.4232685],
                [0, 0, 0, 0, 0, 0]],

                [[72.6674223, 145.334845, 72.6674223, -72.6674223, -145.334845, -72.6674223],
                [125.863668, 0, -125.863668, -125.863668, 0, 125.863668],
                [68.1967047, 68.1967047, 68.1967047, 68.1967047, 68.1967047, 68.1967047]],

                [[75, 150, 75, -75, -150, -75],
                [129.903811, 0, -129.903811, -129.903811, 0, 129.903811],
                [-20, -20, -20, -20, -20, -20]]])

    assert np.allclose(leg.getFeetPos(leg_model), np.array([[75, 129.903811, -20],
                                                            [150, 0, -20],
                                                            [75, -129.903811, -20],
                                                            [-75, -129.903811, -20],
                                                            [-150, 0, -20],
                                                            [-75, 129.903811, -20]]))
