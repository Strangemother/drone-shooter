This application is a Godot game, mostly focused on drone flying. The goal is to produce and support the user as they build out the source.

## Agent Guidelines 

The agent should always aim for the following:

+ Clean, easy to read code
+ Attempt Godot best practices
+ Use the latest Godot version (currently 4.2)
+ Provide comments and documentation for the code

## Important Notes

Where possible do not edit .`tscn` files directly, as the user prefers to make these changes manually within a remote testing computer. Instead, focus on writing GDScript code that can be attached to nodes within the Godot editor.

When providing code snippets, ensure they are complete and can be easily integrated into the existing project structure. Always include necessary imports and class definitions to avoid confusion.

## The User

The user is a long-term educated user in many languages, but is new to Godot 4.x+. They have experience with game development and programming, but are still learning the specifics of Godot's architecture and best practices.

Ensure to focus on providing clear explanations and guidance on how to implement features within the Godot editor, rather than just providing code snippets. This will help the user learn and understand the process of building their game effectively.

## Application Goal

Clear separation of modular code, in a direction to build an agnostic well-behaved all-encompassing drone simulation. It will be used for drone FPV games, and complex flight simulators such as drones, planes, helicopters, and space craft. The key focus is utilising Godot's capabilities to build a flexible platform  for flight simulation, with an emphasis on realistic physics and controls - without the complexity.

---

Continue reading in [docs/](docs/)