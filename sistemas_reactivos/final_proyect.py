#!/usr/bin/env python3

import yaml
import math
import rclpy
import random
import threading
import subprocess
import speech_recognition as sr
import yasmin 
from yasmin_viewer import YasminViewerPub
from yasmin import Blackboard, StateMachine, State, CbState
from yasmin_ros import ActionState
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose

# Resultados de estados
SUCCEED = "succeeded"
ABORT = "aborted"
CONTINUE = "continue"
STOP = "stop"

class ReadWaypointsState(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard) -> str:
        try:
            with open(blackboard["yaml_path"], 'r') as file:
                data = yaml.safe_load(file)
                waypoints_list = data.get("waypoints", [])
                
                blackboard["waypoints"] = {
                    f"wp{i}": self.create_pose(wp) 
                    for i, wp in enumerate(waypoints_list)
                }
                
                yasmin.YASMIN_LOG_INFO("Waypoints disponibles:")
                for name, pose in blackboard["waypoints"].items():
                    yasmin.YASMIN_LOG_INFO(f"- {name}: ({pose.position.x}, {pose.position.y})")
                
                return SUCCEED
        
        except Exception as e:
            yasmin.YASMIN_LOG_INFO(f"Error: {str(e)}")
            return ABORT
        
    def create_pose(self, wp):
        pose = Pose()
        pose.position.x = wp["x"]
        pose.position.y = wp["y"]
        theta = wp["theta"]
        pose.orientation.z = math.sin(theta / 2)
        pose.orientation.w = math.cos(theta / 2)
        return pose

class Nav2State(ActionState):
    def __init__(self):
        super().__init__(
            NavigateToPose,
            "/navigate_to_pose",
            self.create_goal_handler,
            outcomes=["succeeded", "aborted", "canceled"]
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.pose = blackboard["current_pose"]
        goal.pose.header.frame_id = "map"
        return goal
    
    
class FinalAnnouncementState(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED])
        
    def execute(self, blackboard: Blackboard) -> str:
        synthesize_speech("Se ha completado la navegación")
        return SUCCEED


class VoiceControlState(State):
    def __init__(self):
        super().__init__(["continue", "stop"])
        
    def execute(self, blackboard: Blackboard) -> str:
        synthesize_speech("¿Continuar navegación o detener?")
        
        self.command = ""
        def get_command():
            self.command = recognize_command()
            
        thread = threading.Thread(target=get_command)
        thread.start()
        thread.join(timeout=10)
        
        yasmin.YASMIN_LOG_INFO(f"[DEBUG] Comando procesado: '{self.command}'")
        return "stop" if "detener" in self.command else "continue"

def synthesize_speech(text):
    subprocess.call(['espeak', '-v', 'es', text])

def recognize_command():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        yasmin.YASMIN_LOG_INFO("\n[DEBUG] Escuchando... (di algo ahora)")
        try:
            audio = recognizer.listen(source, timeout=5)
        except sr.WaitTimeoutError:
            yasmin.YASMIN_LOG_INFO("[DEBUG] Tiempo de espera agotado")
            return ""
            
    try:
        command = recognizer.recognize_google(audio, language='es-ES').lower()
        yasmin.YASMIN_LOG_INFO(f"[DEBUG] Comando reconocido: '{command}'")
        return command
    except sr.UnknownValueError:
        yasmin.YASMIN_LOG_INFO("[DEBUG] No se entendió el audio")
        return ""
    except Exception as e:
        yasmin.YASMIN_LOG_INFO(f"[DEBUG] Error en reconocimiento: {str(e)}")
        return ""

def get_next_pose(blackboard: Blackboard) -> str:
    try:
        if not blackboard["random_waypoints"]:
            return "completed"
            
        next_wp = blackboard["random_waypoints"].pop(0)
        blackboard["current_pose"] = blackboard["waypoints"][next_wp]
        yasmin.YASMIN_LOG_INFO(f"\nNavegando a: {next_wp}")
        return SUCCEED
        
    except Exception as e:
        yasmin.YASMIN_LOG_INFO(f"Error obteniendo waypoint: {str(e)}")
        return ABORT 
    
def take_random_waypoint(blackboard: Blackboard) -> str:
    blackboard["random_waypoints"] = random.sample(
        list(blackboard["waypoints"].keys()), 
        blackboard["waypoints_num"]
    )
    yasmin.YASMIN_LOG_INFO(f"\nRuta aleatoria seleccionada: {', '.join(blackboard['random_waypoints'])}")
    return SUCCEED

def create_nav_fsm():
    nav_sm = StateMachine(outcomes=[SUCCEED, ABORT])
    
    nav_sm.add_state(
        "GET_NEXT_POSE",
        CbState([SUCCEED, "completed", ABORT], get_next_pose), 
        transitions={
            SUCCEED: "NAVIGATE",
            "completed": "FINAL_ANNOUNCEMENT",
            ABORT: ABORT 
        }
    )
    
    nav_sm.add_state(
        "FINAL_ANNOUNCEMENT",
        FinalAnnouncementState(),
        transitions={SUCCEED: SUCCEED}
    )
    
    nav_sm.add_state(
        "NAVIGATE",
        Nav2State(),
        transitions={
            "succeeded": "VOICE_CHECK",  
            "aborted": ABORT,
            "canceled": ABORT,
        }
    )
    
    nav_sm.add_state(
        "VOICE_CHECK",
        VoiceControlState(),
        transitions={
            "continue": "GET_NEXT_POSE",  
            "stop": "FINAL_ANNOUNCEMENT"
        }
    )
    
    return nav_sm 

def main():
    
    yasmin.YASMIN_LOG_INFO("YASMIN_FINAL_PROYECT")
    
    rclpy.init()
    
    sm = StateMachine(outcomes=[SUCCEED, ABORT])
    
    sm.add_state(
        "LOAD_WAYPOINTS",
        ReadWaypointsState(),
        transitions={SUCCEED: "SELECT_WAYPOINTS", ABORT: ABORT}
    )
    
    sm.add_state(
        "SELECT_WAYPOINTS",
        CbState([SUCCEED], take_random_waypoint),
        transitions={SUCCEED: "NAVIGATION_LOOP"}
    )
    
    sm.add_state(
        "NAVIGATION_LOOP",
        create_nav_fsm(),
        transitions={SUCCEED: SUCCEED, ABORT: ABORT}
    )
    
    blackboard = Blackboard()
    blackboard["yaml_path"] = "/home/aitor/RoboticaCognitiva/src/yasmin/yasmin_demos/yasmin_demos/config_yaml/waypoints.yaml"
    blackboard["waypoints_num"] = 5  
    
    # Se publica la información de la máquina de estados para la visualización
    YasminViewerPub("YASMIN_FINAL_PROYECT", sm)
    
    try:
        outcome = sm(blackboard)
        synthesize_speech("Navegación completada" if outcome == SUCCEED else "Error en navegación")
    except KeyboardInterrupt:
        sm.cancel_state()
        synthesize_speech("Navegación cancelada")
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()