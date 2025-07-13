#!/usr/bin/env python3

import rclpy
from example_interfaces.srv import AddTwoInts

import yasmin
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ServiceState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub


class AddTwoIntsState(ServiceState):
    """
    A state that calls the AddTwoInts service to add two integers.

    This class is a state in a finite state machine that sends a request
    to the AddTwoInts service, retrieves the response, and updates the
    blackboard with the result.

    Attributes:
        service_type (type): The service type being used (AddTwoInts).
        service_name (str): The name of the service.
        outcomes (list): The list of possible outcomes for this state.
    """

    def __init__(self) -> None:
        """
        Initializes the AddTwoIntsState.

        Calls the parent constructor with the specific service type,
        service name, request handler, outcomes, and response handler.
        """
        super().__init__(
            AddTwoInts,  # srv type
            "/add_two_ints",  # service name
            self.create_request_handler,  # cb to create the request
            ["outcome1"],  # outcomes. Includes (SUCCEED, ABORT)
            self.response_handler,  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> AddTwoInts.Request:
        """
        Creates the service request from the blackboard data.

        Args:
            blackboard (Blackboard): The blackboard containing the input values.

        Returns:
            AddTwoInts.Request: The request object populated with values from the blackboard.
        """
        req = AddTwoInts.Request()
        req.a = blackboard["a"]
        req.b = blackboard["b"]
        return req

    def response_handler(
        self, blackboard: Blackboard, response: AddTwoInts.Response
    ) -> str:
        """
        Processes the response from the AddTwoInts service.

        Updates the blackboard with the sum result from the response.

        Args:
            blackboard (Blackboard): The blackboard to update with the sum.
            response (AddTwoInts.Response): The response from the service call.

        Returns:
            str: The outcome of the operation, which is "outcome1".
        """
        blackboard["sum"] = response.sum
        return "outcome1"


def set_ints(blackboard: Blackboard) -> str:
    # Configura múltiples pares de números
    requests = [
        (10, 5),
        (20, 30),
        (100, 200)
    ]
    
    if "requests" not in blackboard:
        blackboard["requests"] = requests
        blackboard["current_index"] = 0
        blackboard["sums"] = []

    # Obtiene el siguiente par de números
    index = blackboard["current_index"]
    if index < len(blackboard["requests"]):
        a, b = blackboard["requests"][index]
        blackboard["a"] = a
        blackboard["b"] = b
        return SUCCEED
    else:
        return ABORT

def store_result(blackboard: Blackboard) -> str:
    # Almacena el resultado y avanza al siguiente
    blackboard["sums"].append(blackboard["sum"])
    blackboard["current_index"] += 1
    return SUCCEED

def print_all_sums(blackboard: Blackboard) -> str:
    yasmin.YASMIN_LOG_INFO(f"All sums: {blackboard['sums']}")
    return SUCCEED


def main():
    """
    The main function to execute the finite state machine (FSM).

    This function initializes the ROS 2 environment, sets up logging,
    creates the FSM with defined states, and executes the FSM.

    Raises:
        KeyboardInterrupt: If the user interrupts the program.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_service_client_demo")

    # Init ROS 2
    rclpy.init()

    # Set ROS 2 logs
    set_ros_loggers()

    # Create a FSM
    sm = StateMachine(outcomes=["outcome4"])

    # Add states
    sm.add_state(
        "SETTING_INTS",
        CbState([SUCCEED, ABORT], set_ints),
        transitions={
            SUCCEED: "ADD_TWO_INTS",
            ABORT: "PRINTING_ALL_SUMS"
        }
    )
    sm.add_state(
        "ADD_TWO_INTS",
        AddTwoIntsState(),
        transitions={"outcome1": "STORING_RESULT"}
    )
    sm.add_state(
        "STORING_RESULT",
        CbState([SUCCEED], store_result),
        transitions={SUCCEED: "SETTING_INTS"}
    )
    sm.add_state(
        "PRINTING_ALL_SUMS",
        CbState([SUCCEED], print_all_sums),
        transitions={SUCCEED: "outcome4"}
    )

    # Publish FSM info
    YasminViewerPub("YASMIN_SERVICE_CLIENT_DEMO", sm)

    # Execute FSM
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    # Shutdown ROS 2
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
