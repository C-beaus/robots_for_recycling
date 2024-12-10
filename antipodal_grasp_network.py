script_dir = os.path.dirname(os.path.abspath(__file__))
package_path = os.path.join(script_dir, "antipodal_grasp_network")
if package_path not in sys.path:
    sys.path.append(package_path)