from ultralytics import YOLO
import torch

# Detectar o número máximo de threads suportadas pela CPU
max_workers = torch.multiprocessing.cpu_count()

# Use um valor moderado, como a metade dos núcleos da CPU
workers = min(max_workers // 2, 8)

def train_model():
    print("Usando GPU:", torch.cuda.is_available())
    print(f"Usando {workers} workers.")
 
    # Carregar o modelo YOLO
    model = YOLO('yolo11x.pt')

    # Configuração do treinamento
    model.train(
        data='data.yaml',       # Dataset e configurações no formato YOLO
        epochs=150,             # Número de épocas de treinamento
        imgsz=640,              # Tamanho das imagens para o treinamento
        batch=8,                # Tamanho do batch
        workers=workers,        # Número de threads para carregamento de dados
        augment=True,           # Habilitar aumentos de dados
        #amp=True,               # Habilitar precisão mista       
    )

    # Salvar o modelo treinado
    model.save('FVBM.pt')

if __name__ == '__main__':
    train_model()
