using LinearAlgebra

#Todo: make this more generalizable to multiple agents

"""
2 Agent
2D Point Mass:
State x: [x, y, ẋ, ẏ]
Input u: [FX, Fy]
"""

function point_mass(x, u)
    c = 0.1     # Damping coefficient [N-s/m]
    m = 1.0     # Mass [kg]
    ẋ₁ = x[3]
    ẍ₁ = -(c/m)*ẋ₁ + u[1]/(m)   
    ẏ₁ = x[4]
    ÿ₁ = -(c/m)*ẏ₁ + u[2]/(m)  
    ẋ₂ = x[7]
    ẍ₂ = -(c/m)*ẋ₂ + u[3]/(m)   
    ẏ₂ = x[8]
    ÿ₂ = -(c/m)*ẏ₂ + u[4]/(m)  
    return [ẋ₁; ẏ₁; ẍ₁; ÿ₁; ẋ₂; ẏ₂; ẍ₂; ÿ₂]
end;

"""
2 Agent
Differential Drive:
State x: [x, y, θ]
Input u: [v, ω]
"""
# function diff_drive(x, u)
#     #x[3] = atan(sin(x[3]),cos(x[3]))
#     ẋ₁ = cos(x[3])*u[1]
#     ẏ₁ = sin(x[3])*u[1]
#     θ̇₁ = u[2]
#     #x[6] = atan(sin(x[6]),cos(x[6]))
#     ẋ₂ = cos(x[6])*u[3]
#     ẏ₂ = sin(x[6])*u[3]
#     θ̇₂ = u[4]

#     return [ẋ₁; ẏ₁; θ̇₁; ẋ₂; ẏ₂; θ̇₂]
# end

function diff_drive(x, u)
    #x[3] = atan(sin(x[3]),cos(x[3]))
    ẋ₁ = cos(x[3])*x[4]
    ẏ₁ = sin(x[3])*x[4]
    θ̇₁ = u[2]
    v̇₁ = u[1]

    #x[6] = atan(sin(x[6]),cos(x[6]))
    ẋ₂ = cos(x[7])*x[8]
    ẏ₂ = sin(x[7])*x[8]
    θ̇₂ = u[4]
    v̇₂ = u[3]

    return [ẋ₁; ẏ₁; θ̇₁; v̇₁; ẋ₂; ẏ₂; θ̇₂; v̇₂]
end