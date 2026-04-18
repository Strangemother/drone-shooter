# Thruster

A Thruster is a more advanced _engine_, a component applied to the vehicle to provide motion. A quick definition

**engine**

A simple force vector, attached to a vehicle, pushing against the world force. The first verison of the engine was a simple force vector. But that will stay pure for future other work.

**thruster**

A thruster is a more complex engine, with its own features and implementation. Fundamentally, a thruster is an engine with a few extra features. Consider this as a SoC with thruster components, and rather than feed it a force value, we feed it _fuel_ and instructions, Like a component in a real car.

The thruster will receive _commands_ through its connections, perhaps have a fuel system, and will output a force vector to the vehicle. The thruster can also have its own internal state, such as temperature, wear and tear, and other factors that can affect its performance.

Under the hood the thruster is a self-contained asset that happens to push against the world force.

## Future 

In the future game we'll have a range of many engines, with their own config and repair schemes. Mounting and alignment will be a key aspect of the game, where micro adjustments perform real changes; like a high-fidelity car game but with futuristic engines.

## Aspects of a Thruster

A fundamental thuster will:

+ Be connected correctly
    + Wired
    + programmed
    + Aligned
    + Fuel attachments
+ Be able to be damaged and repaired
    + With an internal damage system, and a repair system
    + With a fuel system, and a refuel system
+ Be able to be upgraded and modified
    + With a range of upgrades and modifications

The thruster itself responds with a force, and any engine specific events through the connection system. 

--

> Modelling accurate thruster mechanics is a key aspect of the game, and will be a major focus of the development process. The goal is to create a realistic and immersive experience for the player, while also providing a fun and engaging gameplay experience.

## Real Engine Thrusters

We'll use a mixture of drone props, and real engines where appropriate.

- Weight
    And engine has its own weight to append to the overall system
- Output force
    An engine has its own output force, which can be affected by various factors such as fuel quality, damage, and upgrades.
    Typically we can communicate in _newton force per pound_ or anything unspecified - in the future there will be a standardised unit.
- Fuel consumption
    An engine consumes fuel, which can be affected by various factors such as throttle input, damage.
    Electric engines will consume electric charge and have charge sag and other exterior dynamics
    Liquid fuel engines tend to consume in a linear rate
- Damage and wear
    An engine can be damaged, which can affect its performance and fuel consumption.
- Alignment
    An engine's alignment (To very small values!) can affect its performance.

Next more details on force method (exterior affects).

- force area
    The cone of influence of the engine (The stuff coming out the back). It may be a simple cone, or a more complex shape. E.g. Drones are more of a cylinder, while rockets are more of a cone.
- Prop wash
    The force of the engine can affect the world around it, such as blowing dust, or affecting other engines. 
    With drones, prop wash _shudders_ the vehicle. With rockets, the prop wash can affect other vehicles.
- Force Fountain
    An engine close to the ground can create a force fountain, a cushion of air between the engine and the ground. All ground firing engines have this affect.
- Power Ramp
    Some engnes have a delay between when they are commanded to produce force. This may be due to mass, fuel flow, or other factors. 
- Footplate
    An engine close to the ground can create a footplate, a cushion of air between the engine and the ground. This is a more extreme version of the force fountain, where the engine is close enough to the ground to create a solid cushion of air.
- Ground effect
    An engine close to the ground can create a ground effect, where the engine's force is amplified by the ground.
- Airflow
    The engine's force can affect the airflow around it, such as creating turbulence or affecting other engines.
- Heat
    The engine's force can create heat, which can affect the performance of the engine and the vehicle. 
- Sound
    The engine's force can create sound, which can affect the performance of the engine and the vehicle.
- Vibration
    The engine's force can create vibration

Next very advanced features:

- Throttle response
    Some engines have a throttle response, where the engine's force is affected by the throttle input. This can be due to various factors such as fuel flow, mass, and other factors.
- Fuel quality
    The quality of the fuel can affect the performance of the engine, such as the output force and fuel consumption. This can be due to various factors such as impurities in the fuel, or the fuel's energy density.
- Engine tuning
    The engine can be tuned to improve its performance, such as increasing the output force or reducing fuel consumption. This can be done through various methods such as adjusting the fuel flow, or changing the engine's internal components.   
- Engine management system
    An engine management system can be used to control the engine's performance, such as adjusting the fuel flow, or changing the engine's internal components. This can be done through various methods such as a computer system, or a manual control system.
- Engine diagnostics
    An engine diagnostics system can be used to monitor the engine's performance, such as output force, fuel consumption, and other factors. This can be done through various methods such as sensors.
- Engine failure modes
    An engine can have various failure modes, such as a fuel leak, or a mechanical failure. These failure modes can affect the performance of the engine and the vehicle, and can be caused by various factors such as damage, wear and tear, or other factors.
- Engine repair
    An engine can be repaired, which can restore its performance and fuel consumption. 

### Jumbo Jets

As an exmaple, a highlight of focus points for a jumbo jet engine:

- Weight: 10,000 kg
- Output force: 200,000 N
- Fuel consumption: 2,500 kg/hr
- Force area: The cone of influence is approximately 10 degrees.
- Prop wash

### Helicopter

As an exmaple, a highlight of focus points for a helicopter engine:

- Weight: 500 kg
- Output force: 5,000 N
- Fuel consumption: 100 kg/hr
- Force area: The cone of influence is approximately 360 degrees.
- Prop wash
- Force fountain: The helicopter's main rotor creates a force fountain
- Ground effect
- blade stall/bend

### Rocket

As an exmaple, a highlight of focus points for a rocket engine:

- Weight: 1,000 kg
- Output force: 1,000,000 N
- Fuel consumption: 10,000 kg/hr
- Force area: The cone of influence is approximately 5 degrees.
- Large center of mass shift

### Other engines

- Electric engines  
- Plane Propellers
- Drones
- Ion thrusters
- Nuclear engines
- Solar sails




