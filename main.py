"""
=================================================================================================================
                                                Função Principal
=================================================================================================================
"""
import sys
from PyQt6.QtWidgets import QApplication, QMessageBox
from interface import RobotControlInterface


def main():
    """Função principal da aplicação"""
    try:
        app = QApplication(sys.argv)
        app.setApplicationName("Controle Robô RRP")
        app.setApplicationVersion("2.0")
        
        # Estilo melhorado
        app.setStyleSheet("""
            QMainWindow {
                background-color: #f8f9fa;
            }
            
            QGroupBox {
                font-weight: bold;
                border: 2px solid #dee2e6;
                border-radius: 8px;
                margin-top: 1ex;
                padding-top: 15px;
                background-color: white;
            }
            
            QPushButton {
                background-color: #28a745;
                border: none;
                color: white;
                padding: 10px 20px;
                border-radius: 6px;
                font-weight: bold;
            }
            
            QPushButton:hover {
                background-color: #218838;
                border: 2px solid #1e7e34;
            }
            
            QLineEdit {
                padding: 8px;
                border: 2px solid #ced4da;
                border-radius: 4px;
                background-color: white;
            }
            
            QLineEdit:focus {
                border-color: #007bff;
                border-width: 3px;
                background-color: #f0f8ff;
            }
        """)

        # Criar e mostrar janela principal
        window = RobotControlInterface()
        window.show()
        
        # Mensagem de inicialização
        window.update_status("Interface iniciada com sucesso")
        
        # Executar aplicação
        sys.exit(app.exec())
        
    except Exception as e:
        print(f"Erro crítico na aplicação: {e}")
        if 'app' in locals():
            QMessageBox.critical(None, "Erro Crítico", f"Erro ao iniciar aplicação:\n{str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()