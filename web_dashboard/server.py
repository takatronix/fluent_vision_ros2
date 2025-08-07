#!/usr/bin/env python3
from http.server import HTTPServer, SimpleHTTPRequestHandler
import sys

class CORSRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

if __name__ == '__main__':
    port = 8003
    print(f'Starting server on http://localhost:{port}')
    print('With CORS headers enabled')
    print('Press Ctrl+C to stop')
    
    httpd = HTTPServer(('localhost', port), CORSRequestHandler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\nServer stopped')
        sys.exit(0)