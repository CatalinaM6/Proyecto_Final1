import rclpy
from rclpy.node import Node
from proyecto_interfaces.srv import StartPerceptionTest

class Percepcion(Node):
    def __init__(self):
        super().__init__('percep_server')
        self.srv = self.create_service(StartPerceptionTest, '/group_'+str(3)+'/start_perception_test_srv',self.callback)

    def callback(self, request, response):
        # Procesar la solicitud y generar la respuesta
        banner_a = request.banner_a
        banner_b = request.banner_b
        # Lógica para establecer la posición
        # (Aquí puedes agregar tu propia implementación)

        # Supongamos que la posición se establece correctamente
        answer = True

        if answer:
            response.answer = "Posición establecida correctamente"
        else:
            response.answer = "Error al establecer la posición"

        self.get_logger().info(response.answer)
        return response

def main(args=None):
    rclpy.init(args=args)
    server = Percepcion()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
