all: generate

generate:
	python -m grpc_tools.protoc -I../protos --python_out=. --grpc_python_out=. ../protos/robot.proto

clean:
	rm robot_pb2*