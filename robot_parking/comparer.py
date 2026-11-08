"""
comparer.py : regroupe les résultats des évaluations (étape D5) et produit
    - un tableau (Markdown) dans docs/comparaison.md
    - un graphique dans docs/images/D5_comparaison.png
à partir de ~/parking_ws/resultats/eval_*.csv

Utilisation (après avoir évalué chaque conducteur) :
    ros2 run robot_parking comparer
"""
import csv
import glob
import os
from statistics import mean

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

RESULTATS = os.path.expanduser('~/parking_ws/resultats')
DOCS = os.path.expanduser('~/parking_ws/src/robot_parking/docs')
NOMS = {'suiveur': 'A* + suiveur', 'dwa': 'A* + DWA', 'ppo': 'A* + PPO (+ bouclier)'}
ORDRE = ['suiveur', 'dwa', 'ppo']


def lire():
    lignes = []
    for f in glob.glob(os.path.join(RESULTATS, 'eval_*.csv')):
        with open(f) as fichier:
            lignes += list(csv.DictReader(fichier))
    return lignes


def resumer(lignes):
    """Une statistique par (conducteur, avec/sans piéton)."""
    stats = {}
    for c in sorted({l['controleur'] for l in lignes}, key=lambda c: ORDRE.index(c) if c in ORDRE else 9):
        for p in ('0', '1'):
            g = [l for l in lignes if l['controleur'] == c and l['pieton'] == p]
            if not g:
                continue
            ok = [l for l in g if l['reussi'] == '1']
            dmin = [float(l['dist_min_pieton_m']) for l in g if l['dist_min_pieton_m']]
            stats[(c, p)] = {
                'n': len(g),
                'reussite': 100 * len(ok) / len(g),
                'temps': mean(float(l['temps_s']) for l in ok) if ok else float('nan'),
                'distance': mean(float(l['distance_m']) for l in ok) if ok else float('nan'),
                'coll_fixe': sum(int(l['collision_fixe']) for l in g),
                'coll_pieton': sum(int(l['collision_pieton']) for l in g),
                'dmin': min(dmin) if dmin else float('nan'),
            }
    return stats


def tableau(stats):
    t = ['| Conducteur | Piéton | Essais | Réussite | Temps moyen | Distance moyenne | '
         'Collisions voiture/mur | Collisions piéton | Plus près du piéton |',
         '|---|---|---|---|---|---|---|---|---|']
    for (c, p), s in stats.items():
        t.append(f"| {NOMS.get(c, c)} | {'oui' if p == '1' else 'non'} | {s['n']} | "
                 f"{s['reussite']:.0f} % | {s['temps']:.1f} s | {s['distance']:.2f} m | "
                 f"{s['coll_fixe']} | {s['coll_pieton']} | "
                 + ('—' if s['dmin'] != s['dmin'] else f"{s['dmin'] * 100:.0f} cm") + ' |')
    return '\n'.join(t)


def graphique(stats, chemin):
    conducteurs = [c for c in ORDRE if (c, '0') in stats or (c, '1') in stats]
    x = range(len(conducteurs))
    largeur = 0.38
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4))
    for i, (p, nom, couleur) in enumerate([('0', 'sans piéton', '#4C72B0'),
                                           ('1', 'avec piéton', '#DD8452')]):
        decal = [v + (i - 0.5) * largeur for v in x]
        a1.bar(decal, [stats.get((c, p), {}).get('temps', 0) for c in conducteurs],
               largeur, label=nom, color=couleur)
        a2.bar(decal, [stats.get((c, p), {}).get('reussite', 0) for c in conducteurs],
               largeur, label=nom, color=couleur)
    for a, titre, unite in [(a1, 'Temps moyen pour atteindre la place', 's'),
                            (a2, 'Taux de réussite', '%')]:
        a.set_xticks(list(x))
        a.set_xticklabels([NOMS[c] for c in conducteurs], fontsize=9)
        a.set_title(titre, fontsize=11)
        a.set_ylabel(unite)
        a.spines[['top', 'right']].set_visible(False)
        a.legend(frameon=False, fontsize=9)
    a2.set_ylim(0, 105)
    fig.tight_layout()
    fig.savefig(chemin, dpi=130)


def main():
    lignes = lire()
    if not lignes:
        print(f'Aucun fichier eval_*.csv dans {RESULTATS}')
        return
    stats = resumer(lignes)
    t = tableau(stats)
    print(t)
    os.makedirs(os.path.join(DOCS, 'images'), exist_ok=True)
    with open(os.path.join(DOCS, 'comparaison.md'), 'w') as f:
        f.write('# Comparaison des conducteurs (simulation Gazebo)\n\n'
                '6 destinations (A4, B2, B6, C1, C5, D6), sans puis avec un piéton qui traverse '
                'l\'allée devant le robot. Réussite = arrivé en moins de 60 s sans collision. '
                'Temps et distance : moyennes des essais réussis.\n\n' + t +
                '\n\n![comparaison](images/D5_comparaison.png)\n')
    graphique(stats, os.path.join(DOCS, 'images', 'D5_comparaison.png'))
    print(f"\nÉcrit : {os.path.join(DOCS, 'comparaison.md')} et docs/images/D5_comparaison.png")


if __name__ == '__main__':
    main()
