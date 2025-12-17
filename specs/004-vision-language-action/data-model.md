# Data Model: Vision-Language-Action (VLA) Module

## Key Entities

### Voice Command
- **Description**: Natural language instruction spoken by a user, containing semantic intent and parameters for robot action
- **Attributes**:
  - text_content: string (the recognized text)
  - confidence_score: float (0.0-1.0)
  - timestamp: datetime
  - language_code: string
  - intent_classification: string (e.g., "navigation", "manipulation", "query")

### Action Plan
- **Description**: Sequence of specific robot behaviors generated from high-level language instructions
- **Attributes**:
  - plan_id: string (unique identifier)
  - steps: array of action objects
  - status: enum (pending, executing, completed, failed)
  - priority: integer
  - safety_constraints: array of safety requirements

### Vision Feedback
- **Description**: Real-time visual information used to guide and adapt robot actions during execution
- **Attributes**:
  - feedback_type: string (e.g., "object_detection", "depth_map", "semantic_segmentation")
  - timestamp: datetime
  - confidence_scores: array of floats
  - spatial_coordinates: 3D position data
  - processed_data: structured information for action guidance

### Safety Constraint
- **Description**: Predefined limits that ensure robot actions remain safe for humans and environment
- **Attributes**:
  - constraint_type: string (e.g., "distance_limit", "force_limit", "speed_limit")
  - threshold_value: float
  - enforcement_priority: integer
  - override_conditions: array of conditions when constraint may be relaxed

## Relationships

- Voice Command → Action Plan (one-to-many: one command may generate multiple action plans)
- Action Plan → Vision Feedback (many-to-many: action plans use vision feedback for guidance)
- Safety Constraint → Action Plan (many-to-many: multiple constraints apply to each action plan)
- Vision Feedback → Action Plan (one-to-many: one feedback may inform multiple plan adjustments)