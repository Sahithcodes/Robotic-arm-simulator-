/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './src/pages/**/*.{js,ts,jsx,tsx,mdx}',
    './src/components/**/*.{js,ts,jsx,tsx,mdx}',
    './src/app/**/*.{js,ts,jsx,tsx,mdx}',
  ],
  theme: {
    extend: {
      colors: {
        slate: {
          850: '#151e2e',
          950: '#0b0f19',
        },
        engineering: {
          blue: '#2563eb',
          cyan: '#06b6d4',
          amber: '#f59e0b',
          emerald: '#10b981',
          rose: '#f43f5e',
          dark: '#0f172a',
          panel: '#1e293b',
          border: '#334155',
        }
      },
    },
  },
  plugins: [],
}
