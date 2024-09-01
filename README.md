# Pinball

A venture into physics simulations using Box2D as the physics engine and OpenGL as the graphics library.

In particular, the project explores the physical properties of rigid body objects (e.g. density, restitution, etc.), as well as how these properties affect their interactions with one another (i.e. collisions).

<br>

## Features
### Plunger
Spring-loaded, used to launch the ball into play. The longer it's pressed, the stronger it launches the ball.<br>
<p align=center>
<img width=400 src="https://i.giphy.com/media/v1.Y2lkPTc5MGI3NjExZmkwM3g2OGlmaTZqamJhb2liaDN4azRlOG4wbXV3YmkzeTljaHRkbyZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/fFlgq2DULa0ZIrykQi/giphy.gif"/>
</p><br>

### Flippers
Used to launch the ball back into the playfield.<br>
<p align=center>
<img width=400 src="https://i.giphy.com/media/v1.Y2lkPTc5MGI3NjExMnJ0cXpmcHF2OWh4dTY1Y25wZWdhMTIya21tOTAyaGR0eG1pdmpreCZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/LTixAYGXSXNqvoVDsA/giphy.gif"/>
</p><br>

### Obstacles
Various objects (static and dynamic) with which the ball may interact to introduce some variability in play.<br>
<p align=center>
<img width=400 src="https://i.giphy.com/media/v1.Y2lkPTc5MGI3NjExaWJmMWl5dnB0b2VsdDg5OHUzazduaDF3bms0cHg0dTR2dXppMTN2cyZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/aJqAZZKFszaogWyO7c/giphy.gif"/>
</p><br>

### Water
The "game over" area. Simulates the behavior of rigid bodies in fluids depending on their respective densities. This feature also serves as the basis for the next project, [Deformable Body and Particle System Simulation](https://github.com/Hanburgeric/Deformable-Body-and-Particle-System-Simulation).<br>
<p align=center>
<img width=400 src="https://i.giphy.com/media/v1.Y2lkPTc5MGI3NjExbTc2amE0amplanN5dThjaWJ2eDI4a3FoM3h2YzhtODB0Z3Fva3F1ciZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/m6ymqkptetNfXf6U8l/giphy.gif"/>
</p><br>

## To-Do
 - Update code to use modern OpenGL. 
 - Otherwise general refactoring of code to meet current standards.

<br>

## Build and Deployment
In the case that the .exe does not work (normally due to library linking errors), compile and run the code from Visual Studio by following the following steps:
 1. Set active solution configuration to
    <br>`Release`.
 3. Set active solution platform to
    <br>`x64`.
 5. In project properties, under `Debugging`, set `Environment` to
    <br>`PATH=../dll;%PATH$`.
 6. In project properties, under `C/C++`, set `Additional Include Directories` to
    <br>`../include;%(AdditionalIncludeDirectories)`.
 8. In project properties, under `Linker`, set `Additional Library Directories` to
    <br>`../lib/Box2D;../lib/GL;%(AdditionalLibraryDirectories)`.
 10. In project properties, under `Linker` → `Input`, set `Additional Dependencies` to
     <br>`Box2D.lib;Box2D_x86.lib;Box2Dd.lib;Box2Dd_x86.lib;freeglut.lib;freeglutd.lib;glew32.lib;%(AdditionalDependencies)`.

<br>

## Controls
 - `Spacebar` : activates the plunger; hold to increase the amount of force used to launch the ball.
 - `z` : activates the left flipper.
 - `/` : activates the right flipper.

<br>

## Development Environment
 - IDE : Visual Studio
 - Compiler : MSVC
 - Libraries :
   - [Box2D](https://box2d.org/)
   - [FreeGLUT](https://freeglut.sourceforge.net/)
   - [GLEW](https://glew.sourceforge.net/)
