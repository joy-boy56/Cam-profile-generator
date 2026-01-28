from flask import Flask, render_template, request
from cam_profile_generator import CamParameters, CamGeometry

app = Flask(__name__)

@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        params = CamParameters(
            stroke=float(request.form['stroke']),
            base_circle_radius=float(request.form['baseRadius']),
            rise_angle=float(request.form['riseAngle']),
            dwell1_angle=float(request.form['dwell1Angle']),
            return_angle=float(request.form['returnAngle']),
            dwell2_angle=float(request.form['dwell2Angle']),
            motion_law_rise=request.form['motionLawRise'],
            motion_law_return=request.form['motionLawReturn'],
            resolution=360
        )

        total_angle = (
            params.rise_angle +
            params.dwell1_angle +
            params.return_angle +
            params.dwell2_angle
        )

        if abs(total_angle - 360.0) > 1e-6:
            return render_template(
                'index.html',
                error=f"Total cam angle must be 360°. Current total = {total_angle:.2f}°"
            )

        cam = CamGeometry(params)
        cam.calculate_displacement_diagram()
        cam.calculate_pitch_curve()
        x, y = cam.get_profile_coordinates()

        return render_template(
            'result.html',
            x=x.tolist(),
            y=y.tolist()
        )

    return render_template('index.html')


if __name__ == "__main__":
    app.run(debug=True)
