# Quickstart for Contributors

This guide provides instructions for setting up the local development environment and contributing content to the AI & Humanoid Robotics Textbook.

## 1. Prerequisites

-   **Node.js**: Version 20.0 or higher. We recommend using a version manager like `nvm`.
-   **Yarn**: While you can use `npm`, the project is set up with `yarn` lockfiles. Install it via `npm install -g yarn`.
-   **Git**: For version control.
-   **A code editor**: Visual Studio Code is recommended, with the official Docusaurus and Prettier extensions.

## 2. Local Development Setup

1.  **Clone the repository**:
    ```bash
    git clone <repository_url>
    cd <repository_directory>
    ```

2.  **Navigate to the Docusaurus project**:
    All work will be done inside the `docusaurus/` directory.
    ```bash
    cd docusaurus
    ```

3.  **Install dependencies**:
    ```bash
    yarn install
    ```

4.  **Start the development server**:
    ```bash
    yarn start
    ```
    This command starts a local development server, typically at `http://localhost:3000`. The site will automatically reload when you make changes to the source files.

## 3. How to Contribute

### Adding a New Chapter

1.  **Create a Markdown file**: Navigate to the appropriate module folder inside `docusaurus/docs/`. For example, to add a chapter to Module 2, you would go to `docusaurus/docs/module-2/`.
2.  **Name your file**: Use a descriptive, kebab-case name (e.g., `my-new-chapter.md` or `my-new-chapter.mdx` if you need React components).
3.  **Add frontmatter**: Start your file with the standard frontmatter defined in `data-model.md`. This is essential for SEO and metadata.

    ```yaml
    ---
    title: "My New Chapter"
    description: "A summary of my new chapter."
    keywords: ["new keyword"]
    learning_objectives: ["Learn a new thing."]
    prerequisites: ["Module 1"]
    ---

    Your content starts here...
    ```

### Using Custom Components

-   Custom React components are located in `docusaurus/src/components/`.
-   To use a component in an `.mdx` file, import it at the top:
    ```jsx
    import MyComponent from '@site/src/components/MyComponent';

    ## Using a Custom Component

    Here is an example of an interactive component:

    <MyComponent initialValue={5} />
    ```
    Note the use of the `@site` alias, which points to the project root.

### Using Mermaid Diagrams

-   Create a `mermaid` code block in any Markdown file:

    ````markdown
    ```mermaid
    graph TD;
        A-->B;
    ```
    ````

## 4. Submitting Changes

1.  **Create a new branch**:
    ```bash
    git checkout -b feat/your-new-chapter
    ```
2.  **Commit your changes**: Follow the conventional commit message format.
    ```bash
    git add .
    git commit -m "feat(docs): add new chapter on XYZ"
    ```
3.  **Push and open a Pull Request**: Push your branch to the remote repository and open a Pull Request for review.
