module.exports = {
  rootDir: '.',
  testEnvironment: 'jest-environment-jsdom',
  setupFilesAfterEnv: ['<rootDir>/jest.setup.js'],
  moduleNameMapper: {
    '\.css$': 'identity-obj-proxy', // Mocks CSS imports
    '^@site/(.*)$': '<rootDir>/$1',
    '^@docusaurus/(.*)$': '<rootDir>/node_modules/@docusaurus/$1',
    '^@theme/(.*)$': '<rootDir>/node_modules/@docusaurus/theme-classic/lib/theme/$1',
  },
  transform: {
    '^.+\.[jt]sx?$': 'babel-jest',
  },
  testPathIgnorePatterns: ['/node_modules/', '/.docusaurus/'],
};