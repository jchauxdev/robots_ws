"""Practica 5.2 - Clasificador de frutas con PyTorch por TRANSFERENCIA DE APRENDIZAJE.

Red base: MobileNetV3-Small preentrenada en ImageNet (Howard et al., 2019) via torchvision.
Se reemplaza la ultima capa y se ajusta (fine-tuning) sobre un dataset en formato ImageFolder:
    datos/train/<clase>/*.jpg   datos/val/<clase>/*.jpg
Ejemplos de datos: Fruits-360 (Muresan & Oltean, 2018) o imagenes propias (dl_01).

    python3 dl_02_clasificador_transfer.py --datos datos --epocas 10 --congelar
Salidas: clasificador_frutas.pt (pesos + clases), clasificador_frutas.onnx, matriz de confusion.
"""
import argparse
import json
import time

import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from torchvision import datasets, models, transforms

MEDIA, DESV = [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]  # normalizacion ImageNet


def cargadores(raiz, tam, lote):
    t_train = transforms.Compose([
        transforms.RandomResizedCrop(tam, scale=(0.7, 1.0)),
        transforms.RandomHorizontalFlip(),
        transforms.ColorJitter(0.3, 0.3, 0.3, 0.05),
        transforms.ToTensor(), transforms.Normalize(MEDIA, DESV)])
    t_val = transforms.Compose([
        transforms.Resize(int(tam * 1.14)), transforms.CenterCrop(tam),
        transforms.ToTensor(), transforms.Normalize(MEDIA, DESV)])
    tr = datasets.ImageFolder(f"{raiz}/train", t_train)
    va = datasets.ImageFolder(f"{raiz}/val", t_val)
    assert tr.classes == va.classes, "train y val deben tener las mismas clases"
    return (DataLoader(tr, lote, shuffle=True, num_workers=2),
            DataLoader(va, lote, shuffle=False, num_workers=2), tr.classes)


def construir(n_clases, congelar, preentrenado=True):
    w = models.MobileNet_V3_Small_Weights.DEFAULT if preentrenado else None  # descarga ~10 MB (pytorch.org)
    m = models.mobilenet_v3_small(weights=w)
    if congelar:  # solo se entrena el clasificador final (extraccion de caracteristicas)
        for p in m.features.parameters():
            p.requires_grad = False
    m.classifier[3] = nn.Linear(m.classifier[3].in_features, n_clases)
    return m


def evaluar(m, dl, dev, n):
    m.eval()
    conf = torch.zeros(n, n, dtype=torch.long)
    with torch.no_grad():
        for x, y in dl:
            p = m(x.to(dev)).argmax(1).cpu()
            for t, q in zip(y, p):
                conf[t, q] += 1
    acc = conf.diag().sum().item() / max(1, conf.sum().item())
    return acc, conf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--datos", default="datos")
    ap.add_argument("--epocas", type=int, default=10)
    ap.add_argument("--lote", type=int, default=32)
    ap.add_argument("--tam", type=int, default=224)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--congelar", action="store_true")
    ap.add_argument("--semilla", type=int, default=42)
    ap.add_argument("--sin_preentrenar", action="store_true",
                    help="entrena desde cero (sin internet); solo para pruebas, la exactitud sera menor")
    a = ap.parse_args()
    torch.manual_seed(a.semilla)
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print("Dispositivo:", dev, "| torch", torch.__version__)
    dl_tr, dl_va, clases = cargadores(a.datos, a.tam, a.lote)
    print("Clases:", clases)
    m = construir(len(clases), a.congelar and not a.sin_preentrenar, not a.sin_preentrenar).to(dev)
    opt = torch.optim.AdamW([p for p in m.parameters() if p.requires_grad], lr=a.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=a.epocas)
    crit = nn.CrossEntropyLoss(label_smoothing=0.05)
    mejor = -1.0
    for ep in range(1, a.epocas + 1):
        m.train()
        t0, perdida, n = time.time(), 0.0, 0
        for x, y in dl_tr:
            x, y = x.to(dev), y.to(dev)
            opt.zero_grad()
            loss = crit(m(x), y)
            loss.backward()
            opt.step()
            perdida += loss.item() * len(y)
            n += len(y)
        sched.step()
        acc, conf = evaluar(m, dl_va, dev, len(clases))
        print(f"epoca {ep:02d}  perdida={perdida/n:.4f}  exactitud_val={acc*100:.1f} %  ({time.time()-t0:.1f} s)")
        if acc > mejor:
            mejor = acc
            torch.save({"estado": m.state_dict(), "clases": clases, "tam": a.tam}, "clasificador_frutas.pt")
    print("Mejor exactitud de validacion: %.1f %%" % (mejor * 100))
    print("Matriz de confusion (filas = real, columnas = predicho):\n", conf.numpy())
    with open("clases.json", "w") as f:
        json.dump(clases, f)
    m.load_state_dict(torch.load("clasificador_frutas.pt", map_location=dev)["estado"])
    m.eval()
    torch.onnx.export(m.cpu(), torch.randn(1, 3, a.tam, a.tam), "clasificador_frutas.onnx",
                      input_names=["imagen"], output_names=["logits"], opset_version=17)
    print("Exportado clasificador_frutas.onnx (desplegable con TensorRT: trtexec --onnx=... --fp16)")


if __name__ == "__main__":
    main()
