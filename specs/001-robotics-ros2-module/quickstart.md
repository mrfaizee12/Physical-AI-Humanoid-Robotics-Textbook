# Quickstart Guide: Robotic Nervous System (ROS 2) Module

This guide provides instructions for setting up the development environment and interacting with the educational course material locally.

## 1. Prerequisites

Before you begin, ensure you have the following installed:

*   **Node.js**: Version 18.0 or higher. You can download it from [nodejs.org](https://nodejs.org/).
*   **npm**: Node Package Manager, which comes with Node.js.
*   **Git**: For cloning the repository. Download from [git-scm.com](https://git-scm.com/).
*   **Python**: Version 3.8 or higher.
*   **ROS 2**: Follow the official installation guide for your operating system: [docs.ros.org](https://docs.ros.org/en/humble/Installation.html) (adjust ROS 2 distribution as needed).

## 2. Get the Code

Clone the course repository from GitHub:

```bash
git clone [REPOSITORY_URL]
cd [REPOSITORY_NAME]/book
```
*(Replace `[REPOSITORY_URL]` and `[REPOSITORY_NAME]` with actual values once the repository is established.)*

<h2>3. Install Docusaurus Dependencies</h2>
Navigate to the `book` directory and install the required Node.js packages:

```bash
cd book
npm install
```

<h2>4. Run the Course Locally</h2>
Start the Docusaurus development server to view the course in your web browser:

```bash
npm start
```

This will open a new browser tab with the course running locally (usually at `http://localhost:3000`). Any changes you make to the Markdown/MDX files will automatically refresh the browser.

<h2>5. Explore Code Examples</h2>
The executable code examples are located in the `code-examples/` directory at the repository root. Follow the instructions within each example's subdirectory to run them.

For ROS 2 Python examples, ensure your ROS 2 environment is sourced:

```bash
source /opt/ros/[YOUR_ROS2_DISTRO]/setup.bash
```
*(Replace `[YOUR_ROS2_DISTRO]` with your installed ROS 2 distribution, e.g., `humble`, `foxy`.)*

<h2>6. Build for Production (Optional)</h2>
To build a static version of the course for deployment:

```bash
npm run build
```

The static files will be generated in the `build/` directory.
