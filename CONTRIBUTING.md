# Cómo trabajamos en este repositorio

Este documento es corto a propósito. Si algo no está aquí, pregunta al encargado
del proyecto antes de inventártelo.

## Ramas

- `main` está protegida. Nadie empuja directamente a `main`.
- Cada trabajo va en su propia rama, con nombre descriptivo:
  - `feat/deteccion-marcador`  — funcionalidad nueva
  - `fix/lectura-barometro`    — corrección
  - `test/banco-de-empuje`     — pruebas y experimentos
  - `docs/estructura-readme`   — documentación

## Commits

En español, en imperativo y explicando el qué, no el cómo:

```
Añade filtro de mediana a la altura del telémetro
Corrige el orden de los ejes de la IMU
```

Nada de `cambios`, `pruebas2`, `ahora sí`.

## Pull requests

1. Abre la PR contra `main` con la plantilla rellena.
2. La revisión la hace el encargado del proyecto (está en `.github/CODEOWNERS`,
   así que GitHub se la pide solo).
3. Se fusiona cuando hay un visto bueno. Si el cambio es urgente y el encargado
   no está, se avisa por el grupo y se deja constancia en la PR.

## Documentación

El código vive aquí; el relato de cómo se llegó a él, en el Drive:

- **Informe de sesión** — cada día de trabajo, lo escribe quien trabaja.
- **Informe semanal** — lo monta el encargado juntando los informes de sesión.
- **Nota de fase** — al cerrar una fase: dificultades, soluciones, decisiones y resultados.

Si una decisión técnica condiciona el diseño (una librería, un protocolo, un
sensor), déjala escrita también en `docs/` de este repo, aunque sean cinco líneas.

## Lo que nunca entra al repo

- Claves, tokens, contraseñas o credenciales de ningún tipo.
- Registros de vuelo completos y vídeos: van al Drive y se enlazan.
- Material de patrocinadores sujeto a acuerdo sin permiso expreso.

Si se te cuela algo de esto, **avisa en cuanto lo veas**: borrarlo del historial
es más trabajo que borrarlo del disco, y cuanto antes se haga, menos duele.
