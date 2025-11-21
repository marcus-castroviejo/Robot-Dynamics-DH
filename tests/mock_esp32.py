"""
Simulador de ESP32 para testes

Este módulo simula o comportamento de uma ESP32 real, respondendo a comandos
do protocolo e enviando medições. Útil para:
- Desenvolvimento sem hardware
- Testes automatizados
- Demonstrações

Uso:
    # Como servidor (ESP32 se conecta ao PC)
    mock = MockESP32()
    mock.connect_to_server("localhost", 9000)
    mock.start()
    
    # Como mock para testes
    mock = MockESP32()
    mock.configure_trajectory([...])
    measurement = mock.process_command({"cmd": "get_meas"})
"""
import socket
import json
import time
import numpy as np
from typing import List, Dict, Optional, Tuple
import threading


class MockESP32:
    """
    Simulador de ESP32 para testes
    
    Simula:
    - Resposta a comandos (ping, get_meas, set_ref, set_gripper)
    - Geração de medições realistas (com ruído opcional)
    - Atraso de comunicação configurável
    - Dinâmica simples do robô
    """
    
    def __init__(self, noise_level: float = 0.001, delay_ms: int = 10):
        """
        Inicializa o simulador
        
        Args:
            noise_level: Nível de ruído nas medições (padrão: 0.001)
            delay_ms: Atraso de resposta em ms (padrão: 10)
        """
        self.noise_level = noise_level
        self.delay_ms = delay_ms
        
        # Estado atual do robô simulado
        self.current_q = np.array([0.0, 0.0, 0.0])
        self.current_gripper = 0
        
        # Comando recebido
        self.target_q = np.array([0.0, 0.0, 0.0])
        self.target_gripper = 0
        
        # Configuração de dinâmica simples
        self.velocity_factor = 0.1  # Velocidade de convergência
        
        # Socket para comunicação
        self.socket: Optional[socket.socket] = None
        self.connected = False
        self.running = False
        
        # Thread de execução
        self.thread: Optional[threading.Thread] = None
        
        # Tempo simulado
        self.sim_time = 0.0
    
    def configure_position(self, q: List[float], gripper: int = 0):
        """
        Configura posição inicial
        
        Args:
            q: Posição inicial [q1, q2, d3]
            gripper: Posição inicial da garra
        """
        self.current_q = np.array(q, dtype=float)
        self.target_q = self.current_q.copy()
        self.current_gripper = gripper
        self.target_gripper = gripper
    
    def process_command(self, command: Dict) -> Optional[Dict]:
        """
        Processa um comando e retorna resposta
        
        Args:
            command: Dicionário com comando
            
        Returns:
            Dicionário com resposta ou None
        """
        cmd_type = command.get("cmd")
        
        if cmd_type == "ping":
            return {"pong": True}
        
        elif cmd_type == "get_meas":
            # Simula atraso
            time.sleep(self.delay_ms / 1000.0)
            
            # Adiciona ruído
            noisy_q = self.current_q + np.random.normal(0, self.noise_level, 3)
            
            return {
                "meas_q": noisy_q.tolist(),
                "meas_gripper": int(self.current_gripper),
                "t": self.sim_time
            }
        
        elif cmd_type == "set_ref":
            # Atualiza alvo
            q_cmd = command.get("q_cmd")
            gripper_cmd = command.get("gripper")
            
            if q_cmd is not None:
                self.target_q = np.array(q_cmd, dtype=float)
            if gripper_cmd is not None:
                self.target_gripper = int(gripper_cmd)
            
            return {"ref": q_cmd}
        
        elif cmd_type == "set_gripper":
            # Atualiza apenas garra
            value = command.get("value")
            if value is not None:
                self.target_gripper = int(value)
            
            return {"gripper": value}
        
        else:
            return {"error": f"Unknown command: {cmd_type}"}
    
    def update_dynamics(self, dt: float = 0.01):
        """
        Atualiza dinâmica simples do robô
        
        Args:
            dt: Passo de tempo
        """
        # Dinâmica de primeira ordem: q → target_q
        error = self.target_q - self.current_q
        self.current_q += self.velocity_factor * error
        
        # Garra se move mais rápido
        gripper_error = self.target_gripper - self.current_gripper
        self.current_gripper += 0.2 * gripper_error
        
        # Atualiza tempo
        self.sim_time += dt
    
    def connect_to_server(self, host: str = "localhost", port: int = 9000) -> bool:
        """
        Conecta ao servidor PC
        
        Args:
            host: Endereço do servidor
            port: Porta do servidor
            
        Returns:
            True se conectou com sucesso
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((host, port))
            self.connected = True
            print(f"[MockESP32] Conectado a {host}:{port}")
            return True
        except Exception as e:
            print(f"[MockESP32] Erro ao conectar: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Desconecta do servidor"""
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.connected = False
        print("[MockESP32] Desconectado")
    
    def start(self):
        """Inicia loop de comunicação em thread separada"""
        if not self.connected:
            raise RuntimeError("Not connected. Call connect_to_server() first.")
        
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        print("[MockESP32] Loop iniciado")
    
    def _run_loop(self):
        """Loop principal de comunicação (roda em thread)"""
        buffer = b""
        
        while self.running:
            try:
                # Atualiza dinâmica
                self.update_dynamics(dt=0.01)
                
                # Recebe dados
                data = self.socket.recv(1024)
                if not data:
                    print("[MockESP32] Conexão fechada pelo servidor")
                    break
                
                buffer += data
                
                # Processa linhas completas
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    
                    try:
                        # Parse comando
                        command = json.loads(line.decode("utf-8"))
                        print(f"[MockESP32] RX: {command}")
                        
                        # Processa e responde
                        response = self.process_command(command)
                        
                        if response:
                            response_json = json.dumps(response) + "\n"
                            self.socket.send(response_json.encode("utf-8"))
                            print(f"[MockESP32] TX: {response}")
                    
                    except json.JSONDecodeError as e:
                        print(f"[MockESP32] JSON decode error: {e}")
                    except Exception as e:
                        print(f"[MockESP32] Error processing command: {e}")
                
                # Pequeno delay
                time.sleep(0.01)
            
            except Exception as e:
                print(f"[MockESP32] Loop error: {e}")
                break
        
        self.disconnect()
    
    def stop(self):
        """Para o loop de comunicação"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        self.disconnect()


class MockESP32Trajectory:
    """
    Simulador que segue trajetória pré-definida
    
    Útil para testes onde você quer respostas determinísticas.
    """
    
    def __init__(self, trajectory: List[Tuple[float, List[float], int]]):
        """
        Inicializa com trajetória
        
        Args:
            trajectory: Lista de (tempo, q, gripper)
        """
        self.trajectory = trajectory
        self.index = 0
    
    def get_next_measurement(self) -> Optional[Dict]:
        """
        Retorna próxima medição da trajetória
        
        Returns:
            Dicionário com medição ou None se acabou
        """
        if self.index >= len(self.trajectory):
            return None
        
        t, q, gripper = self.trajectory[self.index]
        self.index += 1
        
        return {
            "meas_q": q,
            "meas_gripper": gripper,
            "t": t
        }
    
    def reset(self):
        """Reinicia trajetória"""
        self.index = 0


# =============================================================================
# Função auxiliar para testes
# =============================================================================

def create_mock_esp32_for_testing(initial_q: Optional[List[float]] = None) -> MockESP32:
    """
    Cria MockESP32 configurado para testes
    
    Args:
        initial_q: Posição inicial (padrão: [0, 0, 0])
        
    Returns:
        Instância configurada de MockESP32
    """
    if initial_q is None:
        initial_q = [0.0, 0.0, 0.0]
    
    mock = MockESP32(noise_level=0.0, delay_ms=0)  # Sem ruído/delay para testes
    mock.configure_position(initial_q, gripper=0)
    
    return mock


# =============================================================================
# Script standalone
# =============================================================================

if __name__ == "__main__":
    """
    Roda MockESP32 como processo standalone
    
    Uso:
        python mock_esp32.py
        
    Então inicie a aplicação principal que se conectará a este mock.
    """
    import argparse
    
    parser = argparse.ArgumentParser(description="Mock ESP32 para testes")
    parser.add_argument("--host", default="localhost", help="Host do servidor")
    parser.add_argument("--port", type=int, default=9000, help="Porta do servidor")
    parser.add_argument("--noise", type=float, default=0.001, help="Nível de ruído")
    parser.add_argument("--delay", type=int, default=10, help="Delay em ms")
    
    args = parser.parse_args()
    
    print("="*60)
    print("Mock ESP32 - Simulador de Hardware")
    print("="*60)
    print(f"Host: {args.host}")
    print(f"Port: {args.port}")
    print(f"Noise: {args.noise}")
    print(f"Delay: {args.delay}ms")
    print("="*60)
    
    mock = MockESP32(noise_level=args.noise, delay_ms=args.delay)
    mock.configure_position([0.0, 1.57, 0.03], gripper=0)
    
    print("\nTentando conectar...")
    if mock.connect_to_server(args.host, args.port):
        print("Conectado! Iniciando loop...")
        mock.start()
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\nEncerrando...")
            mock.stop()
    else:
        print("Falha ao conectar. Certifique-se que o servidor está rodando.")