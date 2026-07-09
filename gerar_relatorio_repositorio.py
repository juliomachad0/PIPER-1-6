from pathlib import Path
from datetime import datetime
import os
import textwrap
import html

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (
    SimpleDocTemplate,
    Paragraph,
    Spacer,
    Preformatted,
    PageBreak,
)
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont


# ============================================================
# CONFIGURAÇÕES PRINCIPAIS
# ============================================================

NOME_PDF_FINAL = "relatorio_repositorio.pdf"

# Pasta onde o PDF final sera salvo.
# Como solicitado, o relatorio sera salvo em docs/relatorio_repositorio.pdf.
PASTA_SAIDA_PDF = "docs"

# Apenas estas extensoes terao o CONTEUDO lido e copiado no relatorio.
# Todos os demais arquivos serao apenas listados por nome/caminho.
EXTENSOES_COM_CONTEUDO = {
    ".m",
    ".md",
}

# Escolha quais pastas terao o conteudo dos arquivos .m e .md copiado.
# Exemplos:
#   PASTAS_INCLUIR_CONTEUDO = []
#       Inclui conteudo de todos os arquivos .m e .md encontrados.
#
#   PASTAS_INCLUIR_CONTEUDO = ["navigation", "plots", "xplane"]
#       Inclui conteudo somente dos arquivos .m e .md dentro dessas pastas.
#
# Observacao:
# - A estrutura do repositorio e a lista de arquivos continuam mostrando tudo.
# - Este filtro afeta apenas a secao de conteudo copiado dos arquivos .m e .md.
PASTAS_INCLUIR_CONTEUDO = []

# Se PASTAS_INCLUIR_CONTEUDO nao estiver vazio, arquivos .m/.md na raiz
# normalmente nao entram na secao de conteudo, pois nao pertencem a uma pasta.
# Altere para True se quiser incluir tambem arquivos .m/.md da raiz quando
# estiver usando filtro por pastas.
INCLUIR_ARQUIVOS_RAIZ_QUANDO_FILTRAR = False

# Pastas ignoradas na varredura.
PASTAS_IGNORADAS = {
    ".git",
    ".svn",
    ".hg",
    "__pycache__",
    ".pytest_cache",
    ".mypy_cache",
    ".vscode",
    ".idea",
    "node_modules",
    "venv",
    ".venv",
    "env",
    ".env",
    "build",
    "dist",
}

# Arquivos ignorados.
# O PDF final e o proprio script sao ignorados para evitar autorreferencia.
ARQUIVOS_IGNORADOS = {
    NOME_PDF_FINAL,
    "gerar_relatorio_repositorio.py",
}

# Largura maxima aproximada de linhas de codigo no PDF.
LARGURA_LINHA_CODIGO = 115


# ============================================================
# FUNCOES DE VARREDURA
# ============================================================

def caminho_relativo(caminho: Path, raiz: Path) -> str:
    """
    Retorna caminho relativo usando barra normal, inclusive no Windows.
    """
    return str(caminho.relative_to(raiz)).replace("\\", "/")


def deve_ignorar_pasta(nome_pasta: str) -> bool:
    return nome_pasta in PASTAS_IGNORADAS


def deve_ignorar_arquivo(nome_arquivo: str) -> bool:
    if nome_arquivo in ARQUIVOS_IGNORADOS:
        return True

    # Ignora temporarios do Office.
    if nome_arquivo.startswith("~$"):
        return True

    return False


def listar_arquivos_do_repositorio(raiz: Path):
    """
    Percorre a raiz e subpastas.

    Importante:
    - Para arquivos comuns, coleta apenas nome/caminho.
    - O conteudo so sera lido depois para arquivos .m e .md selecionados.
    """
    arquivos = []

    for dirpath, dirnames, filenames in os.walk(raiz):
        dirpath = Path(dirpath)

        # Remove da recursao as pastas ignoradas.
        dirnames[:] = [
            d for d in dirnames
            if not deve_ignorar_pasta(d)
        ]

        for filename in filenames:
            if deve_ignorar_arquivo(filename):
                continue

            arquivo = dirpath / filename
            arquivos.append(arquivo)

    arquivos = sorted(
        arquivos,
        key=lambda p: caminho_relativo(p, raiz).lower()
    )

    return arquivos


# ============================================================
# FILTRO DE CONTEUDO POR PASTA
# ============================================================

def normalizar_lista_pastas(pastas):
    """
    Normaliza nomes de pastas para comparacao.
    Aceita nomes como 'navigation', 'navigation/' ou 'navigation\\'.
    """
    normalizadas = []

    for pasta in pastas:
        if pasta is None:
            continue

        p = str(pasta).strip().replace("\\", "/").strip("/")

        if p:
            normalizadas.append(p.lower())

    return normalizadas


def arquivo_esta_em_pasta_incluida(arquivo: Path, raiz: Path, pastas_incluir):
    """
    Decide se o conteudo de um arquivo .m/.md deve entrar no relatorio.

    Regras:
    - Se pastas_incluir estiver vazia: inclui todos os .m/.md.
    - Se pastas_incluir tiver valores: inclui somente os arquivos cujo caminho
      relativo esteja dentro de uma dessas pastas.
    - Exemplo: 'navigation/FK/init_EKF_DI.m' entra se 'navigation' estiver na lista.
    """
    pastas_norm = normalizar_lista_pastas(pastas_incluir)

    if not pastas_norm:
        return True

    rel = caminho_relativo(arquivo, raiz).lower()
    partes = rel.split("/")

    # Arquivo na raiz nao tem pasta-pai dentro do caminho relativo.
    if len(partes) == 1:
        return INCLUIR_ARQUIVOS_RAIZ_QUANDO_FILTRAR

    for pasta in pastas_norm:
        if rel == pasta or rel.startswith(pasta + "/"):
            return True

    return False


# ============================================================
# GERACAO DA ESTRUTURA DO REPOSITORIO
# ============================================================

def gerar_arvore_repositorio(raiz: Path, arquivos):
    """
    Gera uma arvore textual da estrutura de pastas e arquivos.
    """
    arvore = {}

    for arquivo in arquivos:
        partes = arquivo.relative_to(raiz).parts
        atual = arvore

        for parte in partes:
            atual = atual.setdefault(parte, {})

    linhas = [f"{raiz.name}/"]

    def escrever_no(no, prefixo=""):
        itens = sorted(
            no.items(),
            key=lambda item: (not bool(item[1]), item[0].lower())
        )

        total = len(itens)

        for indice, (nome, filhos) in enumerate(itens):
            ultimo = indice == total - 1
            conector = "└── " if ultimo else "├── "

            linhas.append(prefixo + conector + nome)

            if filhos:
                novo_prefixo = prefixo + ("    " if ultimo else "│   ")
                escrever_no(filhos, novo_prefixo)

    escrever_no(arvore)

    return "\n".join(linhas)


def gerar_lista_arquivos(raiz: Path, arquivos):
    """
    Gera lista simples e ordenada de arquivos.
    Nao le conteudo dos arquivos.
    """
    linhas = []

    for arquivo in arquivos:
        rel = caminho_relativo(arquivo, raiz)
        linhas.append(rel)

    return "\n".join(linhas)


def gerar_resumo_extensoes(arquivos):
    """
    Gera contagem por extensao.
    Nao le conteudo dos arquivos.
    """
    contagem = {}

    for arquivo in arquivos:
        ext = arquivo.suffix.lower()
        if ext == "":
            ext = "[sem extensão]"

        contagem[ext] = contagem.get(ext, 0) + 1

    linhas = []

    for ext in sorted(contagem.keys()):
        linhas.append(f"{ext}: {contagem[ext]}")

    return "\n".join(linhas)


# ============================================================
# LEITURA APENAS DOS ARQUIVOS .m E .md SELECIONADOS
# ============================================================

def ler_conteudo_texto(arquivo: Path):
    """
    Le somente arquivos .m e .md selecionados.
    Tenta codificacoes comuns para projetos MATLAB e documentacao.
    """
    codificacoes = [
        "utf-8",
        "utf-8-sig",
        "cp1252",
        "latin-1",
    ]

    for codificacao in codificacoes:
        try:
            return arquivo.read_text(encoding=codificacao)
        except UnicodeDecodeError:
            continue
        except Exception as erro:
            return f"[ERRO AO LER O ARQUIVO: {erro}]"

    try:
        return arquivo.read_text(encoding="utf-8", errors="replace")
    except Exception as erro:
        return f"[ERRO AO LER O ARQUIVO: {erro}]"


def selecionar_arquivos_com_conteudo(raiz: Path, arquivos):
    """
    Seleciona apenas .m e .md, em ordem, respeitando PASTAS_INCLUIR_CONTEUDO.
    """
    selecionados = []

    for arquivo in arquivos:
        if arquivo.suffix.lower() not in EXTENSOES_COM_CONTEUDO:
            continue

        if not arquivo_esta_em_pasta_incluida(
            arquivo,
            raiz,
            PASTAS_INCLUIR_CONTEUDO,
        ):
            continue

        selecionados.append(arquivo)

    return selecionados


# ============================================================
# CONFIGURACAO DO PDF
# ============================================================

def registrar_fontes():
    """
    Tenta registrar fontes Unicode para evitar problemas com acentos.
    """
    fonte_texto = "Helvetica"
    fonte_mono = "Courier"

    fontes_possiveis = [
        {
            "nome": "DejaVuSans",
            "caminhos": [
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                "C:/Windows/Fonts/arial.ttf",
                "/System/Library/Fonts/Supplemental/Arial.ttf",
            ],
            "tipo": "texto",
        },
        {
            "nome": "DejaVuSansMono",
            "caminhos": [
                "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
                "C:/Windows/Fonts/consola.ttf",
                "/System/Library/Fonts/Menlo.ttc",
            ],
            "tipo": "mono",
        },
    ]

    for fonte in fontes_possiveis:
        for caminho in fonte["caminhos"]:
            caminho_fonte = Path(caminho)

            if caminho_fonte.exists():
                try:
                    pdfmetrics.registerFont(
                        TTFont(fonte["nome"], str(caminho_fonte))
                    )

                    if fonte["tipo"] == "texto":
                        fonte_texto = fonte["nome"]
                    elif fonte["tipo"] == "mono":
                        fonte_mono = fonte["nome"]

                    break
                except Exception:
                    pass

    return fonte_texto, fonte_mono


def criar_estilos_pdf():
    fonte_texto, fonte_mono = registrar_fontes()

    styles = getSampleStyleSheet()

    estilos = {}

    estilos["titulo"] = ParagraphStyle(
        "Titulo",
        parent=styles["Title"],
        fontName=fonte_texto,
        fontSize=18,
        leading=22,
        alignment=TA_LEFT,
        spaceAfter=14,
    )

    estilos["subtitulo"] = ParagraphStyle(
        "Subtitulo",
        parent=styles["Normal"],
        fontName=fonte_texto,
        fontSize=10,
        leading=13,
        spaceAfter=8,
    )

    estilos["h1"] = ParagraphStyle(
        "H1",
        parent=styles["Heading1"],
        fontName=fonte_texto,
        fontSize=15,
        leading=18,
        spaceBefore=14,
        spaceAfter=8,
    )

    estilos["h2"] = ParagraphStyle(
        "H2",
        parent=styles["Heading2"],
        fontName=fonte_texto,
        fontSize=12,
        leading=15,
        spaceBefore=10,
        spaceAfter=6,
    )

    estilos["normal"] = ParagraphStyle(
        "NormalCustom",
        parent=styles["Normal"],
        fontName=fonte_texto,
        fontSize=9,
        leading=12,
        spaceAfter=4,
    )

    estilos["codigo"] = ParagraphStyle(
        "Codigo",
        fontName=fonte_mono,
        fontSize=7,
        leading=8.6,
        leftIndent=0,
        rightIndent=0,
        spaceBefore=4,
        spaceAfter=8,
    )

    return estilos


def adicionar_paragrafo(story, texto, estilo):
    """
    Adiciona paragrafo escapando caracteres especiais.
    """
    texto_seguro = html.escape(texto)
    story.append(Paragraph(texto_seguro, estilo))


def quebrar_linhas_longas(texto: str, largura: int):
    """
    Quebra linhas longas para evitar estouro lateral no PDF.
    """
    linhas_saida = []

    for linha in texto.splitlines():
        if len(linha) <= largura:
            linhas_saida.append(linha)
        else:
            partes = textwrap.wrap(
                linha,
                width=largura,
                replace_whitespace=False,
                drop_whitespace=False,
            )

            if partes:
                linhas_saida.extend(partes)
            else:
                linhas_saida.append(linha)

    return "\n".join(linhas_saida)


def adicionar_bloco_mono(story, texto, estilo_codigo):
    """
    Adiciona bloco monoespacado no PDF.
    Usado para arvore, listas e codigos.
    """
    texto = quebrar_linhas_longas(texto, LARGURA_LINHA_CODIGO)

    if not texto.strip():
        texto = "[vazio]"

    story.append(Preformatted(texto, estilo_codigo))


# ============================================================
# GERACAO DO RELATORIO PDF
# ============================================================

def obter_caminho_pdf_saida(raiz: Path) -> Path:
    """
    Garante que a pasta docs/ exista e retorna o caminho final do PDF.
    """
    pasta_saida = raiz / PASTA_SAIDA_PDF
    pasta_saida.mkdir(parents=True, exist_ok=True)
    return pasta_saida / NOME_PDF_FINAL


def descricao_filtro_conteudo():
    pastas_norm = normalizar_lista_pastas(PASTAS_INCLUIR_CONTEUDO)

    if not pastas_norm:
        return "Todas as pastas foram consideradas para inclusao de conteudo .m e .md."

    return "Somente estas pastas tiveram conteudo .m e .md incluido: " + ", ".join(pastas_norm)


def gerar_pdf_relatorio(raiz: Path, arquivos):
    caminho_pdf = obter_caminho_pdf_saida(raiz)

    estilos = criar_estilos_pdf()

    doc = SimpleDocTemplate(
        str(caminho_pdf),
        pagesize=A4,
        rightMargin=1.5 * cm,
        leftMargin=1.5 * cm,
        topMargin=1.5 * cm,
        bottomMargin=1.5 * cm,
        title="Relatorio do Repositorio",
        author="Script automatico Python",
    )

    story = []

    data_geracao = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    arquivos_com_conteudo = selecionar_arquivos_com_conteudo(raiz, arquivos)

    # ------------------------------------------------------------
    # CAPA / INTRODUCAO
    # ------------------------------------------------------------

    story.append(Paragraph("Relatório do Repositório", estilos["titulo"]))

    adicionar_paragrafo(
        story,
        f"Pasta raiz analisada: {raiz}",
        estilos["subtitulo"],
    )

    adicionar_paragrafo(
        story,
        f"Data de geração: {data_geracao}",
        estilos["subtitulo"],
    )

    adicionar_paragrafo(
        story,
        "Este relatório foi gerado automaticamente a partir da pasta raiz do projeto. "
        "O relatório lista a estrutura de pastas e arquivos do repositório e, em seguida, "
        "inclui o conteúdo completo apenas dos arquivos com extensão .m e .md selecionados.",
        estilos["normal"],
    )

    story.append(Spacer(1, 10))

    adicionar_paragrafo(
        story,
        f"Total de arquivos listados: {len(arquivos)}",
        estilos["normal"],
    )

    adicionar_paragrafo(
        story,
        f"Total de arquivos .m e .md com conteúdo incluído: {len(arquivos_com_conteudo)}",
        estilos["normal"],
    )

    adicionar_paragrafo(
        story,
        descricao_filtro_conteudo(),
        estilos["normal"],
    )

    adicionar_paragrafo(
        story,
        f"Arquivo PDF salvo em: {caminho_pdf}",
        estilos["normal"],
    )

    # ------------------------------------------------------------
    # ESTRUTURA DO REPOSITORIO
    # ------------------------------------------------------------

    story.append(PageBreak())
    story.append(Paragraph("1. Estrutura de Pastas e Arquivos", estilos["h1"]))

    arvore = gerar_arvore_repositorio(raiz, arquivos)
    adicionar_bloco_mono(story, arvore, estilos["codigo"])

    # ------------------------------------------------------------
    # LISTA ORDENADA DE ARQUIVOS
    # ------------------------------------------------------------

    story.append(PageBreak())
    story.append(Paragraph("2. Lista Ordenada de Arquivos", estilos["h1"]))

    adicionar_paragrafo(
        story,
        "A lista abaixo apresenta os arquivos encontrados no repositório, em ordem alfabética por caminho relativo.",
        estilos["normal"],
    )

    lista_arquivos = gerar_lista_arquivos(raiz, arquivos)
    adicionar_bloco_mono(story, lista_arquivos, estilos["codigo"])

    # ------------------------------------------------------------
    # RESUMO POR EXTENSAO
    # ------------------------------------------------------------

    story.append(PageBreak())
    story.append(Paragraph("3. Resumo por Extensão", estilos["h1"]))

    adicionar_paragrafo(
        story,
        "A contagem abaixo considera apenas nomes e extensões dos arquivos, sem leitura de conteúdo.",
        estilos["normal"],
    )

    resumo_extensoes = gerar_resumo_extensoes(arquivos)
    adicionar_bloco_mono(story, resumo_extensoes, estilos["codigo"])

    # ------------------------------------------------------------
    # CONTEUDO DOS ARQUIVOS .m E .md
    # ------------------------------------------------------------

    story.append(PageBreak())
    story.append(Paragraph("4. Conteúdo dos Arquivos .m e .md", estilos["h1"]))

    adicionar_paragrafo(
        story,
        descricao_filtro_conteudo(),
        estilos["normal"],
    )

    if not arquivos_com_conteudo:
        adicionar_paragrafo(
            story,
            "Nenhum arquivo .m ou .md foi selecionado para inclusão de conteúdo.",
            estilos["normal"],
        )
    else:
        for indice, arquivo in enumerate(arquivos_com_conteudo, start=1):
            rel = caminho_relativo(arquivo, raiz)
            ext = arquivo.suffix.lower()

            story.append(PageBreak())

            story.append(
                Paragraph(
                    f"4.{indice}. {html.escape(rel)}",
                    estilos["h2"],
                )
            )

            adicionar_paragrafo(
                story,
                f"Arquivo: {rel}",
                estilos["normal"],
            )

            adicionar_paragrafo(
                story,
                f"Extensão: {ext}",
                estilos["normal"],
            )

            adicionar_paragrafo(
                story,
                "Conteúdo:",
                estilos["normal"],
            )

            conteudo = ler_conteudo_texto(arquivo)
            adicionar_bloco_mono(story, conteudo, estilos["codigo"])

    doc.build(story)

    return caminho_pdf


# ============================================================
# EXECUCAO PRINCIPAL
# ============================================================

def main():
    raiz = Path.cwd().resolve()

    print("Iniciando geração do relatório...")
    print(f"Pasta raiz: {raiz}")
    print(f"Pasta de saída do PDF: {raiz / PASTA_SAIDA_PDF}")

    pastas_norm = normalizar_lista_pastas(PASTAS_INCLUIR_CONTEUDO)
    if pastas_norm:
        print("Filtro de conteúdo ativo para .m/.md:")
        for pasta in pastas_norm:
            print(f"- {pasta}")
    else:
        print("Filtro de conteúdo vazio: todos os arquivos .m/.md serão considerados.")

    arquivos = listar_arquivos_do_repositorio(raiz)

    print(f"Arquivos encontrados: {len(arquivos)}")

    arquivos_com_conteudo = selecionar_arquivos_com_conteudo(raiz, arquivos)
    print(f"Arquivos .m e .md que terão conteúdo incluído: {len(arquivos_com_conteudo)}")

    caminho_pdf = gerar_pdf_relatorio(raiz, arquivos)

    print("")
    print("Relatório gerado com sucesso:")
    print(caminho_pdf)
    print("")
    print("Concluído.")


if __name__ == "__main__":
    main()
