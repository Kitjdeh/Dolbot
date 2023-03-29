FROM ubuntu:20.04
WORKDIR /home/socket
RUN apt update && apt-get install python3 python3-pip -y
COPY . .
RUN pip install --no-cache-dir -r ./requirements.txt
CMD ["python3","-u","./sio_server.py"]