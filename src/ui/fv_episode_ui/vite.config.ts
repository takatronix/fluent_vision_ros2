import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'
import tailwindcss from '@tailwindcss/vite'

// dev: 5173, build → dist/ (later ament-installed to share/web/).
export default defineConfig({
  plugins: [svelte(), tailwindcss()],
  server: {
    host: '0.0.0.0',
    port: 5173,
    strictPort: true,
    cors: true,
    // LAN dev: allow any host to reach the dev server (mDNS / IP / etc).
    // Production build is served by the dashboard so this setting only
    // affects `npm run dev`.
    allowedHosts: true,
  },
  build: {
    outDir: 'dist',
    emptyOutDir: true,
    rollupOptions: {
      output: {
        entryFileNames: 'assets/[name].js',
        chunkFileNames: 'assets/[name].js',
        assetFileNames: 'assets/[name].[ext]',
      },
    },
  },
  // Relative base so the bundle can be mounted under any subpath
  // by the dashboard (e.g. /static/fv_episode_ui/).
  base: './',
})
