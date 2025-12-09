# Quickstart: Running the Book Locally

This guide provides the steps to build and view the Docusaurus-based book on your local machine. This is useful for previewing changes and ensuring content renders correctly before committing.

## Prerequisites

- **Node.js**: Version 18.x or higher.
- **npm**: Version 9.x or higher (usually installed with Node.js).
- You are in the root directory of the `textbook` repository.

## Setup and Installation

1.  **Navigate to the Book Directory**:
    The Docusaurus project is located in the `book/` subdirectory.

    ```bash
    cd book
    ```

2.  **Install Dependencies**:
    Install all the necessary Node.js packages defined in `package.json`.

    ```bash
    npm install
    ```
    This command will create a `node_modules` directory with all the required libraries.

## Running the Local Development Server

1.  **Start the Docusaurus Server**:
    This command builds the application and starts a local development server with hot-reloading enabled.

    ```bash
    npm run start
    ```

2.  **View the Book**:
    Once the server starts, it will typically open a new browser tab automatically. If not, open your web browser and navigate to:

    [http://localhost:3000](http://localhost:3000)

    You should now see the homepage of the book. As you make changes to the Markdown files in the `docs/` directory, the website will automatically refresh to show your updates.

## Building for Production

To create a static, production-ready build of the site (which is what gets deployed to GitHub Pages), you can run the build command.

1.  **Run the Build Script**:
    This will generate optimized HTML, CSS, and JavaScript files in the `book/build` directory.

    ```bash
    npm run build
    ```

This quickstart is sufficient for previewing and verifying content changes for the book modules.
