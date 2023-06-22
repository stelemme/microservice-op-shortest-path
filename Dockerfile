FROM continuumio/miniconda3

RUN apt-get update
RUN apt-get install -y mono-complete

WORKDIR /app

COPY environment.yml .

RUN conda config --set channel_priority strict

RUN conda env create -f environment.yml 

SHELL ["conda", "run", "-n", "shortest_path", "/bin/bash", "-c"]

ENV FLASK_APP=flaskr
COPY . /app

ENTRYPOINT ["conda", "run", "--no-capture-output", "-n", "shortest_path", "flask", "run", "--host=0.0.0.0", "--port=80"]