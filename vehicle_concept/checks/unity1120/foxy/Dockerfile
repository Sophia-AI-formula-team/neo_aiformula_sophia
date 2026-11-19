ARG BASE_IMAGE=vehicle-semantic:1120
FROM ${BASE_IMAGE}
SHELL ["/bin/bash","-e","-o","pipefail","-c"]
ENV PYTHONUNBUFFERED=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=117
ENV PYTHONPATH=/opt/concept-deps:/vision
WORKDIR /opt/concept
COPY src/ src/
RUN python3 -m pip install --no-index --no-deps --find-links=/tmp/626wheels --target=/opt/concept-deps scipy==1.10.1 && \
    for pkg in concept_road concept_candidates concept_feasibility; do (cd src/$pkg && python3 setup.py build_ext --inplace); done && \
    python3 -c "import scipy, torch, torchvision; assert scipy.__version__ == '1.10.1'"
COPY checks/unity1120/foxy/ /opt/concept/unity/
ENTRYPOINT ["bash","/opt/concept/unity/run.sh"]
