using LinearAlgebra



function solveiLQGame(x₀, xgoal, Q1, Q2, Qn1, Qn2, R11, R12, R21, R22, umin, umax, dmax, ρ, dt, H)

    n = 8
    m1 = 2
    m2 = 2
    k_steps = trunc(Int, H/dt) 
    x̂ = zeros(k_steps, n) # 1500 x need
    û₁ = zeros(k_steps, m1)
    û₂ = zeros(k_steps, m2) 
    P₁ = rand(m1, n, k_steps)*0.01
    P₂ = rand(m2, n, k_steps)*0.01
    α₁ = rand(m1, k_steps)*0.01
    α₂ = rand(m2, k_steps)*0.01

    P = [P₁, P₂]
    α = [α₁, α₂]
    û = [û₁, û₂]

    xₜ, u1ₜ, u2ₜ = rollout_PM(x₀, x̂, û, umin, umax, H, dt, P, α, 0.0) #xₜ is [k_steps, 4]

    Aₜ = zeros(Float32, (n, n, k_steps))
    B1ₜ = zeros(Float32, (n, m1, k_steps))
    B2ₜ = zeros(Float32, (n, m2, k_steps))

    Q1ₜ = zeros(Float32, (n, n, k_steps))
    Q2ₜ = zeros(Float32, (n, n, k_steps))

    l1ₜ = zeros(Float32, (n, k_steps))
    l2ₜ = zeros(Float32, (n, k_steps))

    R11ₜ = zeros(Float32, (m1, m1, k_steps))
    R12ₜ = zeros(Float32, (m1, m2, k_steps))
    R22ₜ = zeros(Float32, (m2, m2, k_steps))
    R21ₜ = zeros(Float32, (m2, m1, k_steps))

    r11ₜ = zeros(Float32, (m1, k_steps))
    r12ₜ = zeros(Float32, (m1, k_steps))
    r22ₜ = zeros(Float32, (m2, k_steps))
    r21ₜ = zeros(Float32, (m2, k_steps))

    m₁ = 1
    m₂ = 1

    A1 = sparse([0 0 1 0; 0 0 0 1; 0 0 (-c/m₁) 0; 0 0 0 (-c/m₁)])
    A2 = sparse([0 0 1 0; 0 0 0 1; 0 0 (-c/m₂) 0; 0 0 0 (-c/m₂)])
    A = blockdiag(A1, A2)
    B1 = sparse([0 0; 0 0; (1/m₁) 0; 0 (1/m₁); 0 0; 0 0; 0 0; 0 0])  #Control Jacobian for point mass 1
    B2 = sparse([0 0; 0 0; 0 0; 0 0; 0 0; 0 0; (1/m₂) 0; 0 (1/m₂)])    #Control Jacobian for point mass 2

    Ad = dt .* A + I    #discretize (zero order hold)
    B1d = dt .*B1   #discrete (zero order hold)
    B2d = dt .*B2;   #discrete (zero order hold)

    converged = false
    u1goal = [0; 0]; u2goal = [0; 0]; 
    βreg = 1.0
    αₗ = [1.0, 0.6, 0.4, 0.2, 0.1, 0.01]
    while !converged
    #for i in 1:50
        converged = isConverged(xₜ, x̂, tol = 1e-6)
        #println(converged)
        total_cost1 = 0
        total_cost2 = 0
        for t = 1:(k_steps-1)
            # 2. Linearize dynamics about trajectory
            
            Aₜ[:,:,t] = Ad
            B1ₜ[:,:,t] = B1d
            B2ₜ[:,:,t] = B2d
            
            #Player 1 cost quadratic_cost(cost_fun, Qi, Rii, Rij, Qni, x, ui, uj, xgoal, uigoal, ujgoal, B)
            costval1, Q1ₜ[:,:,t], l1ₜ[:,t], R11ₜ[:,:,t], r11ₜ[:,t], R12ₜ[:,:,t], r12ₜ[:,t] = 
            quadratic_cost(cost, Q1, R11, R12, Qn1, xₜ[t,:], u1ₜ[t,:], u2ₜ[t,:], xgoal, u1goal, u2goal, dmax, ρ,false)
            #Player 2 cost
            costval2, Q2ₜ[:,:,t], l2ₜ[:,t], R22ₜ[:,:,t], r22ₜ[:,t], R21ₜ[:,:,t], r21ₜ[:,t] = 
            quadratic_cost(cost, Q2, R22, R21, Qn2, xₜ[t,:], u2ₜ[t,:], u1ₜ[t,:], xgoal, u2goal, u1goal, dmax, ρ, false)
            # Regularization
            while !isposdef(Q1ₜ[:,:,t])
                Q1ₜ[:,:,t] = Q1ₜ[:,:,t] + βreg*I
            end
            while !isposdef(Q2ₜ[:,:,t])
                Q2ₜ[:,:,t] = Q2ₜ[:,:,t] + βreg*I
            end
            # @show Rₜ[:,:,t] 
            total_cost1 += costval1
            total_cost2 += costval2
        end
        #Player 1 Terminal cost
        costval1, Q1ₜ[:,:,end], l1ₜ[:,end], R11ₜ[:,:,end], r11ₜ[:,end], R12ₜ[:,:,end], r12ₜ[:,end] = 
        quadratic_cost(cost, Q1, R11, R12, Qn1, xₜ[end,:], u1ₜ[end,:], u2ₜ[end,:], xgoal, u1goal, u2goal, dmax, ρ, true)
        #Player 2 Terminal cost
        costval2, Q2ₜ[:,:,end], l2ₜ[:,end], R22ₜ[:,:,end], r22ₜ[:,end], R21ₜ[:,:,end], r21ₜ[:,end] = 
        quadratic_cost(cost, Q2, R22, R21, Qn2, xₜ[end,:], u2ₜ[end,:], u1ₜ[end,:], xgoal, u2goal, u1goal, dmax, ρ, true)

        total_cost1 += costval1
        total_cost2 += costval2

        # @show costs[end]
        # 4. Do lqr
        #P, α = affinelq!(Aₜ, Bₜ, Qₜ, lₜ, Rₜ , rₜ);
        P₁, P₂, α₁, α₂ = lqGame!(Aₜ, B1ₜ, B2ₜ, Q1ₜ, Q2ₜ, l1ₜ, l2ₜ, R11ₜ, R12ₜ, R21ₜ, R22ₜ, r11ₜ, r22ₜ, r12ₜ, r21ₜ)

        x̂ = xₜ
        û₁ = u1ₜ
        û₂ = u2ₜ 
        P = [P₁, P₂]
        α = [α₁, α₂]
        û = [û₁, û₂]
    
        xₜ, u1ₜ, u2ₜ = rollout_PM(x₀, x̂, û, umin, umax, H, dt, P, α, 0.5)

    end

    return xₜ, u1ₜ, u2ₜ
end