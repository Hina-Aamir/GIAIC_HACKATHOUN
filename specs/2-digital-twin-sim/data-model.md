# Data Model: Module 2 - The Digital Twin (Gazebo & Unity)

## Key Entities

### Digital Twin Model
- **Name**: Unique identifier for the digital twin
- **Description**: Human-readable description of the robot model
- **URDF Path**: File path to the URDF model definition
- **Physics Properties**: Mass, inertia, friction, and collision properties
- **Visual Properties**: Materials, textures, and rendering parameters
- **Sensor Configurations**: List of sensors attached to the model
- **Kinematic Properties**: Joint limits, types, and ranges of motion

### Simulation Environment
- **Name**: Unique identifier for the environment
- **Description**: Human-readable description of the environment
- **World File**: Path to the SDF world file for Gazebo
- **Scene File**: Path to the Unity scene file
- **Physics Parameters**: Gravity, update rate, and solver settings
- **Obstacles**: List of static and dynamic objects in the environment
- **Lighting Conditions**: Environmental lighting and atmospheric settings

### Sensor Data
- **Type**: Sensor type (LiDAR, depth camera, IMU, etc.)
- **Timestamp**: Time of data capture
- **Frame ID**: Coordinate frame of reference
- **Raw Data**: Original sensor measurements
- **Processed Data**: Processed sensor output
- **Noise Model**: Parameters for realistic noise simulation
- **Accuracy Metrics**: Expected accuracy and precision values

### Physics Parameters
- **Gravity**: Gravitational acceleration vector
- **Update Rate**: Physics engine update frequency
- **Solver Type**: Physics solver algorithm (ODE, Bullet, DART)
- **Tolerance**: Numerical tolerance for physics calculations
- **Max Contacts**: Maximum number of contacts per collision
- **Friction Coefficients**: Static and dynamic friction values
- **Restitution**: Coefficient of restitution (bounciness)

### Visualization Properties
- **Materials**: PBR material properties (albedo, normal, roughness, etc.)
- **Lighting**: Light sources, intensities, and colors
- **Camera Settings**: Field of view, resolution, and clipping planes
- **Rendering Quality**: Quality settings for real-time rendering
- **LOD Settings**: Level of detail configurations for performance

## Relationships

### Digital Twin Model → Simulation Environment
- A digital twin model can be instantiated in multiple simulation environments
- Each instantiation may have environment-specific parameters
- The relationship defines how the model behaves in different contexts

### Digital Twin Model → Sensor Data
- A digital twin model produces sensor data through its attached sensors
- The relationship defines the data flow from simulation to output
- Each sensor type has specific data format and properties

### Simulation Environment → Physics Parameters
- Each environment has specific physics configurations
- The relationship defines the physical behavior of the environment
- Parameters can be adjusted for different simulation scenarios

### Sensor Data → Visualization Properties
- Sensor data can be visualized using specific rendering techniques
- The relationship defines how raw data is presented to users
- Visualization properties affect how sensor data is interpreted

## State Transitions

### Simulation States
- **Configuration**: Environment and model parameters are being set up
- **Initialization**: Models and sensors are loaded into the simulation
- **Running**: Simulation is actively computing physics and sensor data
- **Paused**: Simulation is temporarily stopped but state is preserved
- **Stopped**: Simulation has ended and resources are released

### Model States
- **Idle**: Model is loaded but not moving
- **Moving**: Model is executing motion commands
- **Colliding**: Model is in contact with other objects
- **Stable**: Model has reached equilibrium
- **Unstable**: Model is experiencing chaotic motion

## Validation Rules

### Digital Twin Model Validation
- URDF file must be valid and parseable
- All referenced mesh files must exist
- Joint limits must be within reasonable ranges
- Mass and inertia values must be physically plausible

### Sensor Data Validation
- Timestamps must be sequential and reasonable
- Data values must fall within expected ranges
- Frame IDs must correspond to existing coordinate frames
- Accuracy metrics must be consistent with sensor specifications

### Physics Parameters Validation
- Gravity values must be within realistic ranges
- Update rates must be positive and reasonable
- Solver parameters must be within acceptable limits
- Friction coefficients must be non-negative

### Environment Validation
- World files must be valid SDF format
- Scene files must be valid Unity format
- Lighting parameters must be physically plausible
- Obstacle definitions must not cause conflicts