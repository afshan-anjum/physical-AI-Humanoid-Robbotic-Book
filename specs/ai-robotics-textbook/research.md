# Research & Decisions for AI/Robotics Textbook

This document summarizes the research and decisions made to resolve ambiguities in the implementation plan.

## 1. Testing Strategy

**Decision**:
We will use **Jest** and **React Testing Library (RTL)** for testing custom React components created within the `src/components` directory.

**Rationale**:
- **Industry Standard**: Jest and RTL are the de-facto standard for testing React applications, ensuring a large community, extensive documentation, and wide support.
- **Docusaurus Compatibility**: Docusaurus is a React-based application, and its custom components are standard React components. Testing them follows established React patterns. The research confirms this is the recommended approach.
- **User-Centric Testing**: RTL encourages testing components in a way that resembles how users interact with them, which aligns with our goal of creating a user-friendly educational tool.

**Alternatives Considered**:
- **Cypress/Playwright**: These are excellent for end-to-end (E2E) testing but are heavier and more complex than necessary for unit/integration testing of individual components. They can be considered later if full E2E test suites are required.
- **Enzyme**: Previously popular, but now largely superseded by RTL, which provides a more modern and robust testing philosophy.

**Implementation Notes**:
- A `jest.config.js` will be created to handle Docusaurus-specific module aliases (`@theme`, `@docusaurus`) and to mock static asset imports (CSS, images).
- A `babel.config.js` will be configured to transpile JSX and TypeScript.
- A `jest.setup.js` file will be used to import global matchers from `@testing-library/jest-dom`.

## 2. Performance Goals

**Decision**:
The project will target a **Lighthouse score of 90 or higher** across all four categories: Performance, Accessibility, Best Practices, and SEO.

**Rationale**:
- **High Standard**: A score of 90+ ensures a high-quality user experience, which is critical for an educational platform. It signifies fast loading times, accessibility compliance, and adherence to modern web standards.
- **Achievable Target**: Research indicates that well-configured Docusaurus sites can readily achieve scores in the 90-100 range. Docusaurus itself is optimized for performance.
- **Measurable Quality**: This provides a concrete, measurable goal that can be integrated into a CI/CD pipeline to prevent performance regressions.

**Alternatives Considered**:
- **A lower target (e.g., 80+)**: While easier to achieve, it doesn't enforce the level of quality expected for a premier educational resource.
- **Focusing only on the Performance score**: Neglecting Accessibility, Best Practices, and SEO would compromise the overall quality and reach of the textbook.

## 3. CI/CD Build Time Constraints

**Decision**:
The CI/CD pipeline on GitHub Actions should have a **soft target of 5 minutes** and a **hard limit of 10 minutes** for a complete build and deployment.

**Rationale**:
- **Developer Velocity**: Fast feedback loops are crucial. A build time under 5 minutes keeps the development and review process agile.
- **Realistic Expectations**: Research shows that Docusaurus build times are highly dependent on the number of pages. While recent versions are much faster, a very large site can still take time. Starting with a 5-minute target is reasonable for the initial 6 modules.
- **Scalability**: The 10-minute hard limit provides a buffer as the textbook grows. If builds start exceeding this, it will trigger an investigation into build optimization strategies, such as exploring the "Docusaurus Faster" project (Rspack).

**Alternatives Considered**:
- **A stricter limit (e.g., under 3 minutes)**: May be unrealistic as the site scales and could lead to brittle builds failing for minor reasons.
- **No limit**: This is risky, as it allows build times to grow unchecked, leading to slow deployments and a poor developer experience.

## 4. Integration Best Practices

### Algolia DocSearch

**Decision**:
We will use Algolia's free DocSearch service for open-source projects.

**Implementation**:
1.  Apply for DocSearch on the Algolia website once the site is publicly available.
2.  Upon approval, add the provided `appId`, `apiKey`, and `indexName` to `docusaurus.config.js`.
3.  Set `contextualSearch: true` to scope searches to the relevant version/language.

### Mermaid Diagrams

**Decision**:
Mermaid will be enabled via the official `@docusaurus/theme-mermaid` package.

**Implementation**:
1.  Install the package: `npm install @docusaurus/theme-mermaid`.
2.  In `docusaurus.config.js`, set `markdown.mermaid = true` and add the theme to the `themes` array.
3.  Configure light (`neutral`) and dark (`forest`) themes for the diagrams to match the site's styling.
