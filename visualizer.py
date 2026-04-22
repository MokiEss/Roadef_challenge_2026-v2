# run with argument path, instance name and data 
import sys
import os
import webbrowser
import http.server
import threading
import urllib.parse
import time


if __name__ == "__main__":
    global_path = "./challenge-roadef-2026-main/setA/"
    
    print("Number of arguments: ", len(sys.argv))
    print("Arguments: ", sys.argv)
    
    if len(sys.argv) == 3:
        nb_instance = sys.argv[1]
        path_solution = sys.argv[2]
        set_name = "setA"
        
    elif len(sys.argv) == 4:
        nb_instance = sys.argv[1]
        path_solution = sys.argv[2]
        set_name = sys.argv[3]
    else:
        print("Usage: python visualizer.py <id_instance> <solution_path> <set_path>")
        sys.exit(1)
        
    path_net = global_path + set_name + "-" + nb_instance + "-net.json"
    path_scenario = global_path + set_name + "-" + nb_instance + "-scenario.json"
    path_tm = global_path + set_name + "-" + nb_instance + "-tm.json"
    
    print("Visualizing instance: ", nb_instance)
    print("Network file: ", path_net)
    print("Scenario file: ", path_scenario)
    print("Traffic matrix file: ", path_tm)
    print("Solution file: ", path_solution)

    # Start a local HTTP server rooted at the workspace directory
    workspace_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(workspace_dir)

    port = 8765
    handler = http.server.SimpleHTTPRequestHandler

    httpd = http.server.HTTPServer(("127.0.0.1", port), handler)
    server_thread = threading.Thread(target=httpd.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    print(f"HTTP server started at http://127.0.0.1:{port}")

    def to_server_path(rel_path):
        """Convert a path relative to workspace into an absolute server URL path."""
        abs_path = os.path.abspath(rel_path)
        server_path = "/" + os.path.relpath(abs_path, workspace_dir).replace("\\", "/")
        return server_path

    # Build URL with query parameters (only include params whose value is not "none")
    params = {}
    if path_net != "none":
        params["path_net"] = to_server_path(path_net)
    if path_scenario != "none":
        params["path_scenario"] = to_server_path(path_scenario)
    if path_tm != "none":
        params["path_tm"] = to_server_path(path_tm)
    if path_solution != "none":
        params["path_solution"] = to_server_path(path_solution)

    query_string = urllib.parse.urlencode(params)
    url = f"http://127.0.0.1:{port}/web/index.html"
    if query_string:
        url += "?" + query_string

    print(f"Opening: {url}")
    webbrowser.open(url)

    # Keep server alive until interrupted
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        httpd.shutdown()
        print("Server stopped.")

