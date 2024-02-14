#!/usr/bin/env python
class Cliente(object):  # Asegrate de heredar de 'object' para usar el estilo nuevo de super()
    def __init__(self, nombre, correo, telefono):
        self.nombre = nombre
        self.correo = correo
        self.telefono = telefono

    def imprimir_info(self):
        print("Nombre:", self.nombre)
        print("Correo:", self.correo)
        print("Telfono:", self.telefono)


class Persona(Cliente):
    def __init__(self, nombre, correo, telefono, edad):
        super(Persona, self).__init__(nombre, correo, telefono)  # Cambio aqu
        self.edad = edad

    def imprimir_info(self):
        super(Persona, self).imprimir_info()  # Cambio aqu
        print("Edad:", self.edad)


class Empresa(Cliente):
    def __init__(self, nombre, correo, telefono, ruc):
        super(Empresa, self).__init__(nombre, correo, telefono)  # Cambio aqu
        self.ruc = ruc

    def imprimir_info(self):
        super(Empresa, self).imprimir_info()  # Cambio aqu
        print("RUC:", self.ruc)


def main():
    # Crear una instancia de Persona
    persona = Persona("Juan", "juanexample.com", "123456789", 30)
    print("Informacin de la Persona:")
    persona.imprimir_info()
    print()

    # Crear una instancia de Empresa
    empresa = Empresa("Mi Empresa", "infomiempresa.com", "987654321", "12345678901")
    print("Informacin de la Empresa:")
    empresa.imprimir_info()


if __name__ == "__main__":
    main()
