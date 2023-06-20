from flask import Flask

def create_app():
    # Create the app.
    app = Flask(__name__)

    # Add home blueprint to the app.
    from .blueprints import home
    app.register_blueprint(home.bp)

    # Add some-operation blueprint to the app.
    from .blueprints import shortest_path
    app.register_blueprint(shortest_path.bp)

    return app