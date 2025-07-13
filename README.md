# ROBÓTICA COGNITIVA: prácticas para el Máster en Robótica e Inteligencia Artificial

El siguiente [enlace](https://github.com/ULE-MURIA-24-25/robotica-cognitiva-labs-AigarciabFabero) nos dirige al repositorio principal:

## [Práctica 1: Sistemas reactivos](./sistemas_reactivos/Readme_1.md)

- **Objetivo:**

  Estudiar el uso de máquinas de estados como mecanismos para la implementación de arquitecturas reactivas para la toma de decisiones en robots.

- **Tareas Clave:**

  - Reconocer las diferencias entre mecanismos ad-hoc y sistemas predefinidos para la toma de decisiones con robots.

  - Reconocer las limitaciones de las aproximaciones basadas en máquinas de estados.

  - Familiarizarse con el uso de Yet Another State MachINe YASMIN

  - Desarrollar soluciones de toma de decisiones en robots con YASMIN y ROS 2.
- **Herramientas:** 
  - ROS 2 Humble
  - Ubuntu 22.04
  - YASMIN *https://github.com/uleroboticsgroup/yasmin*

## [Práctica 2: Sistemas jerárquicos](./sistemas_jerarquicos/Readme_2.md)
- **Objetivo:**

  Comprender la estructura de dominios y problemas en PDDL definiendo acciones con precondiciones y efectos para modelar sistemas robóticos. Resolver problemas de planificación mediante herramientas automáticas externas. Integrar los planes generados en entornos simulados o robóticos simples para visualizar su ejecución, y aplicar estos conceptos a escenarios prácticos como la navegación entre puntos clave (waypoints) o la gestión de recursos limitados (por ejemplo, energía).

- **Tareas clave:**

  - Analizar dominios y problemas existentes en PDDL, explicando las acciones disponibles y su utilidad para alcanzar los objetivos.

  - Extender dominios añadiendo nuevas acciones y ajustando los problemas para requerir su uso, generando y justificando nuevos planes.

  - Integrar PDDL en entornos simulados, observando y comentando la correspondencia entre las acciones planificadas y los comportamientos del robot.

  - Desarrollar un caso de navegación entre varios puntos de interés (waypoints), modelando el dominio y problema en PDDL, ejecutando el plan y documentando el resultado.

  - Adaptar los modelos para incluir condiciones adicionales (obstáculos, estados del robot, prioridades, energía, etc.).

- **Herraminetas:**
  - Ubuntu 22.04
  - Python 3.10+
  - PDDL *https://github.com/fjrodl/PDDL-course*
  - PlanSys *https://plansys2.github.io/getting_started/index.html*