function T_IE = jointToHomogenousTr(q)
T_IE = getTransformI0*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*jointToTransform45(q)*jointToTransform56(q)*getTransform6E;
end

