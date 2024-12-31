# URDF Writer

The **URDF Writer** is a Python package designed to help users easily generate Unified Robot Description Format (URDF) files. It supports a command-line interface (CLI) and provides functionality for prompting user input to describe a robot, saving the description in JSON format, and generating the corresponding URDF file. The tool is designed with test-driven development (TDD) principles.

## Features

- **Prompt-Based Robot Description**: 
  - Interactive user prompts to define robot links, joints, visuals, and more.
  - Saves the robot description as a JSON file.

- **URDF Generation**:
  - Converts the JSON robot description into a valid URDF file.
  - Supports defining link visuals (box, cylinder) and materials.
  - Handles joints and origins with error handling for unsupported configurations.

- **Command-Line Interface (CLI)**:
  - Accepts JSON input files to generate URDF files.
  - Specify the output URDF file path.

## Installation

1. Clone the repository:

    ```bash
    git clone <repository-url>
    cd urdf_writer
    ```

2. Install the dependencies:

    ```bash
    pip install -r requirements.txt
    ```

## Usage

### Prompt-Based Robot Description

Run the interactive prompt to describe your robot:

```bash
python -m urdf_writer.robot_prompt
```

Follow the prompts to define your robot. The description will be saved as a JSON file.

### Generate URDF from JSON

Use the CLI to convert a JSON description into a URDF file:

```bash
python -m urdf_writer.cli <input-file.json> --output <output-file.urdf>
```

Example:

```bash
python -m urdf_writer.cli robot_description.json --output robot.urdf
```

### Options

- `--output`: Path to the generated URDF file.

## Development

This project is developed with test-driven development (TDD). All functionality is backed by unit tests.

### Running Tests

To run the tests, use:

```bash
python -m unittest discover tests
```
