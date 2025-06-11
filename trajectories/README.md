python3 -m venv trajectory
source trajectory/bin/activate
pip install -r requirements.txt

python -m ipykernel install --user --name=trajectory

jupyter notebook --ip 0.0.0.0 --port 8888
