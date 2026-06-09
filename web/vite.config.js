import { defineConfig } from 'vite';
import { viteSingleFile } from 'vite-plugin-singlefile';

export default defineConfig({
  base: './',
  plugins: [viteSingleFile()],
  build: {
    outDir: 'dist',
    // Inline every asset regardless of size so nothing is emitted as a
    // separate file alongside index.html.
    assetsInlineLimit: 100000000,
    // No code-splitting: everything ends up in one chunk that the
    // single-file plugin inlines into index.html.
    cssCodeSplit: false,
    rollupOptions: {
      output: {
        inlineDynamicImports: true,
      },
    },
  },
});
